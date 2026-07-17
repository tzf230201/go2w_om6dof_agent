import http.client
import threading
from types import SimpleNamespace
from urllib.parse import urlencode

import pytest

import application_web_monitor.web_monitor as web_monitor


class _Logger:
    def __init__(self):
        self.infos = []
        self.warnings = []
        self.errors = []

    def info(self, message, **_kwargs):
        self.infos.append(str(message))

    def warn(self, message, **_kwargs):
        self.warnings.append(str(message))

    def error(self, message, **_kwargs):
        self.errors.append(str(message))


def _bare_monitor():
    node = object.__new__(web_monitor.MonitorNode)
    node._arm_restart_lock = threading.Lock()
    node._arm_restart_phase = "idle"
    node._arm_restart_message = ""
    node._arm_restart_started = 0.0
    node._controller_state_lock = threading.Lock()
    node._controller_states = {}
    node._controller_states_updated = 0.0
    node._controller_query = None
    node._controller_query_started = 0.0
    node._controller_query_generation = 0
    node.controller_list_client = object()
    node.robot_ssh_host = ""
    node._logger = _Logger()
    node.get_logger = lambda: node._logger
    return node


def test_service_status_uses_fixed_systemctl_query_and_parses_result(monkeypatch):
    calls = []

    def fake_run(argv, **kwargs):
        calls.append((argv, kwargs))
        return SimpleNamespace(
            stdout="ActiveState=active\nSubState=running\nMainPID=321\n",
            stderr="",
            returncode=0,
        )

    monkeypatch.setattr(web_monitor.subprocess, "run", fake_run)

    status = web_monitor.om6dof_service_status()

    assert status == {
        "active_state": "active",
        "sub_state": "running",
        "main_pid": 321,
    }
    argv, kwargs = calls[0]
    assert argv == [
        "/usr/bin/systemctl",
        "show",
        "om6dof-hardware.service",
        "--property=ActiveState",
        "--property=SubState",
        "--property=MainPID",
        "--no-pager",
    ]
    assert "shell" not in kwargs
    assert kwargs["check"] is False
    assert kwargs["timeout"] == 2.0


def test_restart_uses_only_the_sudoers_whitelisted_command(monkeypatch):
    calls = []

    def fake_run(argv, **kwargs):
        calls.append((argv, kwargs))
        return SimpleNamespace(stdout="", stderr="", returncode=0)

    monkeypatch.setattr(web_monitor.subprocess, "run", fake_run)

    result = web_monitor.invoke_om6dof_service_restart()

    assert result.returncode == 0
    argv, kwargs = calls[0]
    assert tuple(argv) == web_monitor.OM6DOF_RESTART_COMMAND
    assert "shell" not in kwargs
    assert kwargs["check"] is False
    assert kwargs["timeout"] == 8.0


def test_restart_worker_reports_ready_only_after_new_pid_and_nodes(monkeypatch):
    node = _bare_monitor()
    statuses = iter([
        {"active_state": "active", "sub_state": "running", "main_pid": 10},
        {"active_state": "active", "sub_state": "running", "main_pid": 20},
    ])
    monkeypatch.setattr(
        web_monitor, "om6dof_service_status", lambda: next(statuses)
    )
    monkeypatch.setattr(
        web_monitor,
        "invoke_om6dof_service_restart",
        lambda: SimpleNamespace(stdout="", stderr="", returncode=0),
    )
    node.arm_stack_missing_nodes = lambda: []
    invalidations = []
    node._invalidate_controller_states = lambda: invalidations.append(True)
    node.arm_controller_issues = lambda: []

    node._restart_arm_stack_worker()

    snapshot = node.arm_restart_snapshot()
    assert snapshot["phase"] == "ready"
    assert "10 -> 20" in snapshot["message"]
    assert invalidations == [True]
    assert node._logger.errors == []


def test_restart_worker_explains_missing_sudoers_rule(monkeypatch):
    node = _bare_monitor()
    monkeypatch.setattr(
        web_monitor,
        "om6dof_service_status",
        lambda: {"active_state": "active", "sub_state": "running", "main_pid": 10},
    )
    monkeypatch.setattr(
        web_monitor,
        "invoke_om6dof_service_restart",
        lambda: SimpleNamespace(
            stdout="", stderr="sudo: a password is required", returncode=1
        ),
    )

    node._restart_arm_stack_worker()

    snapshot = node.arm_restart_snapshot()
    assert snapshot["phase"] == "failed"
    assert "sudoers" in snapshot["message"]
    assert "password is required" in snapshot["message"]


def test_duplicate_restart_request_is_deduplicated(monkeypatch):
    node = _bare_monitor()
    started_threads = []

    class _Thread:
        def __init__(self, **kwargs):
            started_threads.append(kwargs)

        def start(self):
            pass

    monkeypatch.setattr(web_monitor.threading, "Thread", _Thread)

    first_started, _ = node.request_arm_stack_restart()
    second_started, second_message = node.request_arm_stack_restart()

    assert first_started is True
    assert second_started is False
    assert "already in progress" in second_message
    assert len(started_threads) == 1


def test_controller_health_requires_broadcasters_and_exactly_one_arm_owner(
    monkeypatch,
):
    node = _bare_monitor()
    monkeypatch.setattr(web_monitor.time, "monotonic", lambda: 100.0)

    node._controller_states = {
        "joint_state_broadcaster": "active",
        "gripper_controller": "active",
        "arm_controller": "active",
        "forward_position_controller": "inactive",
    }
    node._controller_states_updated = 99.0
    assert node.arm_controller_issues() == []

    node._controller_states["forward_position_controller"] = "active"
    assert node.arm_controller_issues() == [
        "arm_controller=active, forward_position_controller=active"
    ]

    node._controller_states["forward_position_controller"] = "inactive"
    node._controller_states["gripper_controller"] = "inactive"
    assert node.arm_controller_issues() == ["gripper_controller=inactive"]


@pytest.mark.skipif(
    web_monitor.ListControllers is None,
    reason="controller_manager_msgs is optional on the application host",
)
def test_stuck_controller_query_is_removed_and_retried(monkeypatch):
    node = _bare_monitor()
    old_future = object()
    new_future = SimpleNamespace(add_done_callback=lambda _callback: None)

    class _Client:
        def __init__(self):
            self.removed = []
            self.requests = []

        def remove_pending_request(self, future):
            self.removed.append(future)

        def service_is_ready(self):
            return True

        def call_async(self, request):
            self.requests.append(request)
            return new_future

    node.controller_list_client = _Client()
    node._controller_query = old_future
    node._controller_query_started = 90.0
    monkeypatch.setattr(web_monitor.time, "monotonic", lambda: 100.0)

    node._poll_controller_states()

    assert node.controller_list_client.removed == [old_future]
    assert len(node.controller_list_client.requests) == 1
    assert node._controller_query is new_future
    assert node._controller_query_started == 100.0
    assert node._controller_query_generation == 1
    assert "timed out" in node._logger.warnings[-1]


def test_csrf_comparison_safely_rejects_non_ascii():
    assert web_monitor.csrf_token_matches("known-token", "known-token") is True
    assert web_monitor.csrf_token_matches("é", "known-token") is False


def test_forwarded_camera_is_available_only_while_frames_are_recent(monkeypatch):
    now = [100.0]
    monkeypatch.setattr(web_monitor.time, "monotonic", lambda: now[0])
    stream = web_monitor.ForwardedImageStream("/camera/forwarded")

    assert stream.available() is False
    stream.on_image(SimpleNamespace(data=b"jpeg-frame"))
    assert stream.available() is True

    now[0] += stream.ACTIVE_TIMEOUT_S + 0.1
    assert stream.available() is False


def test_forwarded_camera_snapshot_returns_only_a_recent_frame(monkeypatch):
    now = [100.0]
    monkeypatch.setattr(web_monitor.time, "monotonic", lambda: now[0])
    stream = web_monitor.ForwardedImageStream("/camera/forwarded")
    assert stream.snapshot() is None
    stream.on_image(SimpleNamespace(data=b"jpeg-frame"))
    assert stream.snapshot() == b"jpeg-frame"
    now[0] += stream.ACTIVE_TIMEOUT_S + 0.1
    assert stream.snapshot() is None


@pytest.mark.parametrize("message", [
    "what do you see",
    "describe the camera view",
    "is there a person in front of you",
    "how many people are there",
])
def test_visual_questions_are_routed_to_the_camera(message):
    assert web_monitor.is_vision_question(message) is True


def test_motion_command_is_not_misrouted_to_camera():
    assert web_monitor.is_vision_question("move forward one meter") is False


def test_forwarded_pcm_is_available_only_while_frames_are_recent(monkeypatch):
    now = [100.0]
    monkeypatch.setattr(web_monitor.time, "monotonic", lambda: now[0])
    stream = web_monitor.ForwardedPcmStream("/audio/pcm")

    assert stream.available() is False
    stream.on_pcm(SimpleNamespace(data=b"\x00\x00" * 960))
    assert stream.available() is True

    now[0] += stream.ACTIVE_TIMEOUT_S + 0.1
    assert stream.available() is False


def test_audio_http_endpoint_is_absent_without_forwarded_pcm():
    node = _bare_monitor()
    camera = web_monitor.ForwardedImageStream("/camera/forwarded")
    audio = web_monitor.ForwardedPcmStream("/audio/pcm")
    server = web_monitor.ThreadingHTTPServer(
        ("127.0.0.1", 0), web_monitor.make_handler(node, camera, audio)
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    connection = http.client.HTTPConnection(*server.server_address, timeout=2)
    try:
        connection.request("GET", "/audio.pcm")
        response = connection.getresponse()
        response.read()
        assert response.status == 503
    finally:
        connection.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_live_audio_script_has_an_explicit_browser_toggle():
    assert "toggleLiveAudio()" in web_monitor.SCRIPTS
    assert "stopLiveAudio()" in web_monitor.SCRIPTS
    assert "STT remains active" in web_monitor.SCRIPTS
    assert "VOICE DETECTED" in web_monitor.SCRIPTS


def test_camera_http_endpoint_is_absent_without_forwarded_frames():
    node = _bare_monitor()
    stream = web_monitor.ForwardedImageStream("/camera/forwarded")
    server = web_monitor.ThreadingHTTPServer(
        ("127.0.0.1", 0), web_monitor.make_handler(node, stream)
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    connection = http.client.HTTPConnection(*server.server_address, timeout=2)
    try:
        connection.request("GET", "/camera.mjpg")
        response = connection.getresponse()
        response.read()
        assert response.status == 503
    finally:
        connection.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_restart_http_endpoint_rejects_invalid_csrf_and_accepts_valid_token():
    node = _bare_monitor()
    node.csrf_token = "known-token"
    calls = []
    node.request_arm_stack_restart = lambda: (
        calls.append(True) or (True, "restart requested")
    )
    node.flash = ""
    server = web_monitor.ThreadingHTTPServer(
        ("127.0.0.1", 0), web_monitor.make_handler(node, SimpleNamespace())
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    connection = http.client.HTTPConnection(*server.server_address, timeout=2)
    headers = {"Content-Type": "application/x-www-form-urlencoded"}
    try:
        connection.request(
            "POST", "/restart_om6dof", urlencode({"csrf": "wrong"}), headers
        )
        response = connection.getresponse()
        response.read()
        assert response.status == 403
        assert calls == []

        connection.request(
            "POST",
            "/restart_om6dof",
            urlencode({"csrf": "known-token"}),
            headers,
        )
        response = connection.getresponse()
        response.read()
        assert response.status == 303
        assert calls == [True]
    finally:
        connection.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_restart_http_endpoint_rejects_oversized_form_without_action():
    node = _bare_monitor()
    node.csrf_token = "known-token"
    calls = []
    node.request_arm_stack_restart = lambda: (
        calls.append(True) or (True, "restart requested")
    )
    server = web_monitor.ThreadingHTTPServer(
        ("127.0.0.1", 0), web_monitor.make_handler(node, SimpleNamespace())
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    connection = http.client.HTTPConnection(*server.server_address, timeout=2)
    try:
        body = "x" * (web_monitor.MAX_FORM_BODY_BYTES + 1)
        connection.request(
            "POST",
            "/restart_om6dof",
            body,
            {"Content-Type": "application/x-www-form-urlencoded"},
        )
        response = connection.getresponse()
        response.read()
        assert response.status == 413
        assert calls == []
    finally:
        connection.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
