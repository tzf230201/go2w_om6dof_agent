import http.client
import json
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


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


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
    node._pickup_lock = threading.Lock()
    node.pickup_busy = False
    node.pickup_message = "not run yet"
    node.object_tracking_active = False
    node.object_tracking_busy = False
    node.object_tracking_message = "not run yet"
    node.object_search_active = False
    node.object_search_busy = False
    node.object_search_message = "not run yet"
    node.remote_enabled = False
    node.control_mode = "JOINT"
    node.arm_target_active = False
    node.arm_target_state = "idle"
    node.arm_target_mode = ""
    node.arm_target_request_id = ""
    node.arm_target_message = "no target yet"
    node.arm_target_goal = None
    node.arm_target_current = {}
    node.pub_arm_target = _Publisher()
    node.pub_web_jog = _Publisher()
    node.perception_distance_m = 0.3
    node._logger = _Logger()
    node.get_logger = lambda: node._logger
    return node


def test_joint_state_cache_keeps_finite_measured_positions_and_reports_age():
    node = _bare_monitor()
    node._joint_state_lock = threading.Lock()
    node._joint_positions = {}
    node._joint_state_updated = 0.0

    node._on_joint_state(SimpleNamespace(
        name=["joint1", "joint2", "invalid"],
        position=[0.25, -0.5, float("nan")],
    ))
    snapshot = node.joint_state_snapshot()

    assert snapshot["positions"] == {"joint1": 0.25, "joint2": -0.5}
    assert snapshot["available"] is True
    assert snapshot["age_s"] is not None
    assert 0.0 <= snapshot["age_s"] < 1.0


def test_web_jog_is_bounded_and_requires_matching_streaming_mode():
    node = _bare_monitor()
    node.remote_enabled = True
    node.control_mode = "CARTESIAN"

    accepted, message = node.request_web_jog("CARTESIAN", 0, 1.0, -0.5)

    assert accepted is True
    assert message == ""
    assert list(node.pub_web_jog.messages[-1].data) == [
        0.03, -0.015, 0.0, 0.0, 0.0, 0.0,
    ]

    accepted, message = node.request_web_jog("JOINT", 0, 0.0, 0.0)

    assert accepted is False
    assert "not JOINT" in message
    assert len(node.pub_web_jog.messages) == 1


def test_battery_gauge_unknown_state_is_visual_and_accessible():
    markup = web_monitor.battery_pill(SimpleNamespace(battery_soc=None))

    assert 'class="battery-gauge battery-unknown"' in markup
    assert 'style="width:0%"' in markup
    assert 'aria-label="Battery data unavailable"' in markup
    assert '<strong class="battery-percent">--%</strong>' in markup


@pytest.mark.parametrize(
    ("soc", "level", "state_class"),
    [
        (-5, 0, "battery-low"),
        (19, 19, "battery-low"),
        (20, 20, "battery-medium"),
        (39, 39, "battery-medium"),
        (40, 40, "battery-good"),
        (100, 100, "battery-good"),
        (130, 100, "battery-good"),
    ],
)
def test_battery_gauge_clamps_fill_and_uses_level_colour(
    soc, level, state_class,
):
    markup = web_monitor.battery_pill(SimpleNamespace(
        battery_soc=soc,
        battery_v=30.7,
        battery_a=1.9,
    ))

    assert f'class="battery-gauge {state_class}"' in markup
    assert f'style="width:{level}%"' in markup
    assert f'<strong class="battery-percent">{level}%</strong>' in markup
    assert f'aria-label="Battery {level}% · 30.7 V · +1.9 A"' in markup
    visible_text = "".join(
        fragment.split("<", 1)[0] for fragment in markup.split(">")[1:]
    )
    assert visible_text == f"{level}%"


def test_ram_gauge_unknown_state_is_visual_and_accessible():
    markup = web_monitor.ram_gauge({"ram": "?", "ram_percent": None})

    assert 'class="ram-gauge ram-unknown"' in markup
    assert 'style="width:0%"' in markup
    assert 'aria-label="RAM data unavailable"' in markup
    assert '<span class="ram-label">RAM</span>' in markup
    assert '<strong class="ram-percent">--%</strong>' in markup


@pytest.mark.parametrize(
    ("percent", "level", "state_class"),
    [
        (-5, 0, "ram-good"),
        (69, 69, "ram-good"),
        (70, 70, "ram-medium"),
        (84, 84, "ram-medium"),
        (85, 85, "ram-high"),
        (100, 100, "ram-high"),
        (130, 100, "ram-high"),
    ],
)
def test_ram_gauge_clamps_fill_and_uses_pressure_colour(
    percent, level, state_class,
):
    markup = web_monitor.ram_gauge({
        "ram": "8.5 / 64.3 GB",
        "ram_percent": percent,
    })

    assert f'class="ram-gauge {state_class}"' in markup
    assert f'style="width:{level}%"' in markup
    assert f'<strong class="ram-percent">{level}%</strong>' in markup
    assert '<span class="ram-label">RAM</span>' in markup
    assert (
        f'aria-label="RAM usage {level}% · 8.5 / 64.3 GB"'
        in markup
    )
    visible_text = "".join(
        fragment.split("<", 1)[0] for fragment in markup.split(">")[1:]
    )
    assert visible_text == f"RAM {level}%"


def test_battery_gauge_styles_are_embedded_without_external_icon_dependency():
    for selector in (
        ".battery-gauge", ".battery-icon", ".battery-fill",
        ".battery-percent", ".battery-good", ".battery-medium",
        ".battery-low", ".battery-unknown", ".header-battery",
        ".header-clock", ".ram-gauge", ".ram-chip", ".ram-fill",
        ".ram-reading", ".ram-label",
        ".ram-percent", ".ram-good", ".ram-medium", ".ram-high",
        ".ram-unknown", ".header-ram",
    ):
        assert selector in web_monitor.CSS
    assert "position:sticky" in web_monitor.CSS
    assert "function updateHeaderClock()" in web_monitor.SCRIPTS
    assert "setInterval(updateHeaderClock,1000)" in web_monitor.SCRIPTS


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


@pytest.mark.parametrize("action", ["start", "stop"])
def test_perception_control_uses_only_allowlisted_systemctl_commands(
    monkeypatch, action,
):
    calls = []

    def fake_run(argv, **kwargs):
        calls.append((argv, kwargs))
        return SimpleNamespace(stdout="", stderr="", returncode=0)

    monkeypatch.setattr(web_monitor.subprocess, "run", fake_run)

    result = web_monitor.invoke_perception_service(
        action, "unitree@192.168.123.18"
    )

    assert result.returncode == 0
    argv, kwargs = calls[0]
    assert argv[:6] == [
        "/usr/bin/ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=2",
        "unitree@192.168.123.18",
    ]
    assert tuple(argv[6:]) == web_monitor.OM6DOF_PERCEPTION_COMMANDS[action]
    assert "sudo" not in argv
    assert "shell" not in kwargs
    assert kwargs["check"] is False
    assert kwargs["timeout"] == 12.0


def test_perception_control_rejects_unknown_action_without_running_command(
    monkeypatch,
):
    calls = []
    monkeypatch.setattr(
        web_monitor.subprocess, "run", lambda *args, **kwargs: calls.append(args)
    )

    with pytest.raises(ValueError, match="unsupported perception action"):
        web_monitor.invoke_perception_service("restart")

    assert calls == []


@pytest.mark.parametrize("action", ["start", "stop"])
def test_ddgng_control_uses_only_allowlisted_systemctl_commands(
    monkeypatch, action,
):
    calls = []

    def fake_run(argv, **kwargs):
        calls.append((argv, kwargs))
        return SimpleNamespace(stdout="", stderr="", returncode=0)

    monkeypatch.setattr(web_monitor.subprocess, "run", fake_run)

    result = web_monitor.invoke_ddgng_service(
        action, "unitree@192.168.123.18"
    )

    assert result.returncode == 0
    argv, kwargs = calls[0]
    assert argv[:6] == [
        "/usr/bin/ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=2",
        "unitree@192.168.123.18",
    ]
    assert tuple(argv[6:]) == web_monitor.OM6DOF_DDGNG_COMMANDS[action]
    assert "sudo" not in argv
    assert "shell" not in kwargs
    assert kwargs["check"] is False


def test_ddgng_control_rejects_unknown_action(monkeypatch):
    calls = []
    monkeypatch.setattr(
        web_monitor.subprocess, "run", lambda *args, **kwargs: calls.append(args)
    )

    with pytest.raises(ValueError, match="unsupported DD-GNG action"):
        web_monitor.invoke_ddgng_service("restart")

    assert calls == []


def test_perception_status_queries_the_remote_user_service(monkeypatch):
    calls = []

    def fake_run(argv, **kwargs):
        calls.append(argv)
        return SimpleNamespace(
            stdout="ActiveState=inactive\nSubState=dead\nMainPID=0\n",
            stderr="",
            returncode=0,
        )

    monkeypatch.setattr(web_monitor.subprocess, "run", fake_run)

    status = web_monitor.perception_service_status("unitree@robot")

    assert status["active_state"] == "inactive"
    assert calls[0][6:11] == [
        "/usr/bin/systemctl", "--machine=kublab@", "--user", "show",
        "om6dof-perception.service",
    ]


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


def test_perception_status_exposes_object_to_eoe_distance():
    node = _bare_monitor()
    message = web_monitor.String(data=(
        '{"target":{"state":"tracking"},'
        '"ee":{"state":"tracking"},"distance_m":0.4123}'
    ))

    node._on_perception_status(message)

    assert node.perception_tracking_status == "target=tracking, EoE=tracking"
    assert node.perception_distance_m == pytest.approx(0.4123)


def test_pickup_request_is_rejected_while_remote_owns_arm():
    node = _bare_monitor()
    node.remote_enabled = True
    node.pickup_client = SimpleNamespace(service_is_ready=lambda: True)

    started, message = node.request_perception_pick()

    assert started is False
    assert "F3" in message


def test_stuck_pickup_status_request_is_removed_and_retried(monkeypatch):
    node = _bare_monitor()
    old_future = SimpleNamespace(done=lambda: False)
    callbacks = []
    new_future = SimpleNamespace(
        done=lambda: False,
        add_done_callback=callbacks.append,
    )

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

    node.pickup_status_client = _Client()
    node._pickup_status_future = old_future
    node._pickup_status_future_started = 90.0
    node._pickup_status_generation = 3
    monkeypatch.setattr(web_monitor.time, "monotonic", lambda: 100.0)

    node._poll_pickup_status()

    assert node.pickup_status_client.removed == [old_future]
    assert len(node.pickup_status_client.requests) == 1
    assert node._pickup_status_future is new_future
    assert node._pickup_status_future_started == 100.0
    assert node._pickup_status_generation == 5
    assert len(callbacks) == 1
    assert "timed out" in node._logger.warnings[-1]


def test_terminal_pickup_status_clears_busy_state():
    node = _bare_monitor()
    node.pickup_busy = True
    node.pickup_message = "direct pick started"
    node._pickup_status_generation = 7
    response = SimpleNamespace(
        success=False,
        message="step: pickup aborted — no safe IK chain",
    )
    future = SimpleNamespace(result=lambda: response)

    node._on_pickup_status_response(future, 7)

    assert node.pickup_busy is False
    assert "aborted" in node.pickup_message


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


def test_control_forms_are_submitted_without_page_reload():
    assert "event.preventDefault()" in web_monitor.SCRIPTS
    assert "submitControlForm(form,event.submitter)" in web_monitor.SCRIPTS
    assert "'X-Requested-With':'fetch'" in web_monitor.SCRIPTS
    assert "refreshStatusBurst()" in web_monitor.SCRIPTS


def test_camera_http_endpoint_is_absent_without_forwarded_frames():
    node = _bare_monitor()
    stream = web_monitor.ForwardedImageStream("/camera/forwarded")
    node.perception_camera = web_monitor.ForwardedImageStream(
        "/camera/perception"
    )
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
        connection.request("GET", "/perception.mjpg")
        response = connection.getresponse()
        response.read()
        assert response.status == 503
    finally:
        connection.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_camera_streams_use_distinct_ros_topics_and_http_routes():
    assert (
        "/application_web_monitor/image/compressed"
        != "/application_web_monitor/perception/image/compressed"
    )
    assert "/camera.mjpg" in web_monitor.SCRIPTS
    assert "/perception.mjpg" in web_monitor.SCRIPTS


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


def test_restart_http_endpoint_returns_json_for_ajax_without_redirect():
    node = _bare_monitor()
    node.csrf_token = "known-token"
    node.flash = ""
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
    headers = {
        "Content-Type": "application/x-www-form-urlencoded",
        "Accept": "application/json",
        "X-Requested-With": "fetch",
    }
    try:
        connection.request(
            "POST",
            "/restart_om6dof",
            urlencode({"csrf": "known-token"}),
            headers,
        )
        response = connection.getresponse()
        payload = json.loads(response.read().decode("utf-8"))
        assert response.status == 200
        assert response.getheader("Location") is None
        assert payload == {
            "ok": True,
            "message": "restart requested",
            "refresh_ms": 250,
        }
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


def test_perception_http_controls_require_csrf_and_accept_target(monkeypatch):
    node = _bare_monitor()
    node.csrf_token = "known-token"
    node.flash = ""
    actions = []
    targets = []
    monkeypatch.setattr(
        web_monitor,
        "invoke_perception_service",
        lambda action, ssh_host="": (
            actions.append((action, ssh_host))
            or SimpleNamespace(stdout="", stderr="", returncode=0)
        ),
    )
    node.set_perception_target = targets.append
    server = web_monitor.ThreadingHTTPServer(
        ("127.0.0.1", 0),
        web_monitor.make_handler(node, SimpleNamespace()),
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    connection = http.client.HTTPConnection(*server.server_address, timeout=2)
    headers = {"Content-Type": "application/x-www-form-urlencoded"}
    try:
        connection.request(
            "POST", "/start_perception", urlencode({"csrf": "wrong"}), headers
        )
        response = connection.getresponse()
        response.read()
        assert response.status == 403
        assert actions == []

        connection.request(
            "POST",
            "/start_perception",
            urlencode({"csrf": "known-token"}),
            headers,
        )
        response = connection.getresponse()
        response.read()
        assert response.status == 303
        assert actions == [("start", "")]

        connection.request(
            "POST",
            "/target_perception",
            urlencode({"csrf": "known-token", "target": "red cup"}),
            headers,
        )
        response = connection.getresponse()
        response.read()
        assert response.status == 303
        assert targets == ["red cup"]
    finally:
        connection.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_arm_target_command_is_atomic_and_stop_is_always_available(monkeypatch):
    node = _bare_monitor()
    node.remote_enabled = True
    monkeypatch.setattr(web_monitor.secrets, "token_hex", lambda _size: "fixed-id")

    started, message = node.request_arm_target(
        "CARTESIAN", [0.2, 0.0, 0.3, 0.0, 1.0, 0.0]
    )
    assert started is True
    assert "CARTESIAN" in message
    assert json.loads(node.pub_arm_target.messages[-1].data) == {
        "action": "move",
        "mode": "CARTESIAN",
        "values": [0.2, 0.0, 0.3, 0.0, 1.0, 0.0],
        "request_id": "fixed-id",
    }
    assert node.arm_target_active is True

    started, _ = node.request_arm_target_stop()
    assert started is True
    assert json.loads(node.pub_arm_target.messages[-1].data) == {
        "action": "stop", "request_id": "fixed-id",
    }


@pytest.mark.parametrize(
    ("attribute", "value", "expected"),
    [
        ("remote_enabled", False, "enable remote"),
        ("control_mode", "READY", "READY"),
        ("pickup_busy", True, "pickup"),
        ("object_tracking_active", True, "tracking"),
        ("object_search_busy", True, "search is running"),
        ("arm_target_active", True, "another target"),
    ],
)
def test_arm_target_rejects_conflicting_state_without_publish(
    attribute, value, expected
):
    node = _bare_monitor()
    node.remote_enabled = True
    setattr(node, attribute, value)

    started, message = node.request_arm_target("JOINT", [0.0] * 6)
    assert started is False
    assert expected.lower() in message.lower()
    assert node.pub_arm_target.messages == []


@pytest.mark.parametrize(
    ("mode", "values"),
    [
        ("BAD", [0.0] * 6),
        ("JOINT", [0.0] * 5),
        ("JOINT", [0.0, 0.0, 0.0, 0.0, 0.0, float("nan")]),
        ("JOINT", [0.0, 0.0, 0.0, 0.0, 0.0, float("inf")]),
    ],
)
def test_arm_target_rejects_invalid_numbers(mode, values):
    node = _bare_monitor()
    node.remote_enabled = True
    started, _ = node.request_arm_target(mode, values)
    assert started is False
    assert node.pub_arm_target.messages == []


def test_arm_target_status_callback_exposes_live_feedback_and_terminal_state():
    node = _bare_monitor()
    node._on_arm_target_status(SimpleNamespace(data=json.dumps({
        "active": True,
        "state": "running",
        "mode": "JOINT",
        "request_id": "abc",
        "message": "moving",
        "goal": [0.1] * 6,
        "current": {
            "joint": [0.0] * 6,
            "cartesian": [0.2, 0.0, 0.3, 0.0, 1.0, 0.0],
        },
    })))
    assert node.arm_target_active is True
    assert node.arm_target_state == "running"
    assert node.arm_target_current["joint"] == [0.0] * 6

    node._on_arm_target_status(SimpleNamespace(data=json.dumps({
        "active": False,
        "state": "reached",
        "mode": "JOINT",
        "request_id": "abc",
        "message": "target reached",
        "current": {"joint": [0.1] * 6},
    })))
    assert node.arm_target_active is False
    assert node.arm_target_state == "reached"


def test_arm_target_idle_status_uses_english_default_message():
    node = _bare_monitor()
    node._on_arm_target_status(SimpleNamespace(data=json.dumps({
        "active": False,
        "state": "idle",
        "message": "legacy localized text",
        "current": {},
    })))

    assert node.arm_target_active is False
    assert node.arm_target_state == "idle"
    assert node.arm_target_message == "no target yet"


def test_arm_target_http_endpoint_requires_csrf_and_returns_ajax_json():
    node = _bare_monitor()
    node.csrf_token = "known-token"
    node.flash = ""
    calls = []
    stops = []
    node.request_arm_target = lambda mode, values: (
        calls.append((mode, values)) or (True, "target accepted")
    )
    node.request_arm_target_stop = lambda: (
        stops.append(True) or (True, "target stopped")
    )
    server = web_monitor.ThreadingHTTPServer(
        ("127.0.0.1", 0),
        web_monitor.make_handler(node, SimpleNamespace()),
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    connection = http.client.HTTPConnection(*server.server_address, timeout=2)
    ajax_headers = {
        "Content-Type": "application/x-www-form-urlencoded",
        "Accept": "application/json",
        "X-Requested-With": "fetch",
    }
    target_form = {
        "csrf": "known-token", "mode": "JOINT",
        **{f"value_{index}": str(index / 10.0) for index in range(1, 7)},
    }
    try:
        bad_form = dict(target_form, csrf="wrong")
        connection.request(
            "POST", "/arm_target", urlencode(bad_form), ajax_headers
        )
        response = connection.getresponse()
        response.read()
        assert response.status == 403
        assert calls == []

        connection.request(
            "POST", "/arm_target", urlencode(target_form), ajax_headers
        )
        response = connection.getresponse()
        payload = json.loads(response.read().decode("utf-8"))
        assert response.status == 200
        assert response.getheader("Location") is None
        assert payload["ok"] is True
        assert calls == [("JOINT", [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])]

        connection.request(
            "POST", "/arm_target_stop",
            urlencode({"csrf": "known-token"}), ajax_headers,
        )
        response = connection.getresponse()
        response.read()
        assert response.status == 200
        assert stops == [True]

        invalid_form = dict(target_form, value_6="nan")
        connection.request(
            "POST", "/arm_target", urlencode(invalid_form), ajax_headers
        )
        response = connection.getresponse()
        response.read()
        assert response.status == 400
        assert len(calls) == 1
    finally:
        connection.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
