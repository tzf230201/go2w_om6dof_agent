"""Network-accessible web dashboard for the Go2W robot.

A dependency-free (stdlib http.server) dashboard you can open from any device
on the same network. It shows:

  * Robot status  — hostname, IPs, uptime, load, RAM, CPU temp, ROS_DOMAIN_ID,
                    whether /dev/ttyUSB0 (the U2D2 arm bus) is present, and
                    whether remote control currently owns the arm interfaces.
  * Nodes         — the live ROS 2 graph node list, with DUPLICATE node names
                    highlighted (two nodes with the same name = trouble).
  * Topics        — the live topic list with message types.
  * Camera        — an optional MJPEG view of compressed images forwarded over
                    ROS. The card only appears while forwarded frames arrive;
                    this process never opens the camera device itself.
  * Remote toggle — a button that publishes a momentary F3 tap onto
                     /wirelesscontroller. F3 switches controller ownership
                     between autonomous and remote control.
  * Arm targets   — live JOINT, CARTESIAN, and CYLINDRICAL absolute-target
                    forms with current-feedback fill, controller status, and
                    a target-only stop button.
  * OM6DOF restart — a guarded, asynchronous restart of the single systemd
                     unit that owns bringup, the command converter, and teleop.
  * Perception    — guarded start/stop controls for OM6DOF perception on the
                    robot host; its processed stream appears in Camera.

Status is refreshed in place once per second and control forms use AJAX, so
normal operation does not reload the page. The camera is a separate live
stream.

Run:
    ros2 run application_web_monitor web_monitor
    # then open http://<robot-ip>:8080  from a phone/laptop on the same net
Options via ROS params: port (default 8080), camera_topic, and
perception_camera_topic.
"""

from __future__ import annotations

import base64
import html
import json
import math
import os
import re
import secrets
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import List, Optional, Tuple
from urllib.parse import parse_qs

import rclpy
try:
    from controller_manager_msgs.srv import ListControllers
except ImportError:  # Optional on an application-only host such as the AGX.
    ListControllers = None
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from unitree_go.msg import WirelessController, LowState
from std_msgs.msg import Bool, String, UInt8MultiArray
from std_srvs.srv import Trigger
from sensor_msgs.msg import CompressedImage

# unitree_api is only needed for the "Robot Agent" chat mode (direct motion
# control). Import it softly so the dashboard still runs on machines that lack it.
try:
    from unitree_api.msg import Request as SportRequest
except Exception:  # pragma: no cover
    SportRequest = None


BTN_F3 = 1 << 7
ARM_BUS_DEVICE = "/dev/ttyUSB0"
OM6DOF_SERVICE = "om6dof-hardware.service"
OM6DOF_PERCEPTION_SERVICE = "om6dof-perception.service"
OM6DOF_PICK_SERVICE = "om6dof-perception-pick.service"
OM6DOF_DDGNG_SERVICE = "om6dof-dd-gng.service"
PICKUP_STATUS_TIMEOUT_S = 3.0
OM6DOF_REQUIRED_NODES = frozenset({
    "controller_manager",
    "om6dof_controller",
    "om6dof_teleop",
    "robot_state_publisher",
})
OM6DOF_ALWAYS_ACTIVE_CONTROLLERS = (
    "joint_state_broadcaster",
    "gripper_controller",
)
OM6DOF_ARM_CONTROLLERS = (
    "arm_controller",
    "forward_position_controller",
)
OM6DOF_RESTART_COMMAND = (
    "/usr/bin/sudo",
    "-n",
    "/usr/bin/systemctl",
    "--no-block",
    "restart",
    OM6DOF_SERVICE,
)
OM6DOF_PERCEPTION_COMMANDS = {
    "start": (
        "/usr/bin/systemctl", "--user", "start",
        OM6DOF_PERCEPTION_SERVICE, OM6DOF_PICK_SERVICE,
    ),
    "stop": (
        "/usr/bin/systemctl", "--user", "stop",
        OM6DOF_PICK_SERVICE, OM6DOF_PERCEPTION_SERVICE,
    ),
}
OM6DOF_DDGNG_COMMANDS = {
    "start": (
        "/usr/bin/systemctl", "--user", "start", OM6DOF_DDGNG_SERVICE,
    ),
    "stop": (
        "/usr/bin/systemctl", "--user", "stop", OM6DOF_DDGNG_SERVICE,
    ),
}

# Unitree sport-mode API ids (verified against the official Unitree ROS2
# ros2_sport_client.h / .cpp and go2w_cmd_vel_control_node.cpp).
SPORT_TOPIC = "/api/sport/request"
API_DAMP = 1001            # emergency stop — all joints damp (highest priority)
API_BALANCE_STAND = 1002
API_STOP_MOVE = 1003
API_STAND_UP = 1004
API_STAND_DOWN = 1005      # lie down
API_RECOVERY_STAND = 1006  # recover from a fall / lying to balanced stand
API_MOVE = 1008
API_SWITCH_GAIT = 1011     # {"data":d}  0/1/2
API_SPEED_LEVEL = 1015     # {"data":level}  -1/0/1

# Locomotion mode -> SwitchGait "data" code (Go2W: 0 default, 1 stair/terrain
# walking, 2 height-climbing — per the official Go2W high-level control doc).
GAIT_CODES = {"normal": 0, "terrain": 1, "climb": 2}
# Speed gear -> SpeedLevel "data" value. Only effective in the default gait (0).
SPEED_LEVELS = {"slow": -1, "normal": 0, "fast": 1}

# Conservative motion caps for chat-driven commands. Must match the ranges
# documented in skills/go2w_control_skill.md.
MAX_VX = 0.4
MAX_VY = 0.3
MAX_WZ = 0.8
MAX_DURATION = 8.0
MAX_FORM_BODY_BYTES = 4096
CONTROLLER_QUERY_TIMEOUT = 2.5

VISION_SYSTEM = (
    "You are the visual perception assistant for a Unitree Go2W robot. "
    "Answer the user's question using only the attached current camera frame. "
    "Answer in one short sentence of at most 25 words, using plain English "
    "suitable for speech. Mention uncertainty when the image is unclear. "
    "Do not use markdown, lists, or JSON."
)
VISION_PATTERNS = (
    r"\bwhat (?:do|can) you see\b",
    r"\bwhat(?:'s| is) (?:in|on) (?:the )?(?:camera|image|frame|front)\b",
    r"\b(?:look|looking) (?:at|through) (?:the )?camera\b",
    r"\bdescribe (?:the |your )?(?:camera|image|view|scene|surroundings)\b",
    r"\b(?:camera|visual|vision) (?:view|image|result|feed)\b",
    r"\b(?:do you|can you) see\b",
    r"\bhow many (?:people|persons|objects|chairs|boxes|robots)\b",
    r"\b(?:is|are) there (?:a |an |any )?(?:person|people|object|obstacle)\b",
    r"\bwhat is (?:in front of|ahead of) (?:you|the robot)\b",
)


# --------------------------------------------------------------------------- #
#  Node → OS process mapping, for the per-node kill button                     #
# --------------------------------------------------------------------------- #
def find_node_pids(name: str) -> List[int]:
    """Best-effort map a ROS 2 node name to the PID(s) hosting it by scanning
    /proc cmdlines. Preference order: an explicit `__node:=<name>` remap, then a
    matching executable basename, then a bare token match. Never returns our own
    PID (so the dashboard can't kill itself)."""
    me = os.getpid()
    exact: List[int] = []
    exe: List[int] = []
    token: List[int] = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == me:
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                raw = f.read()
        except Exception:
            continue
        if not raw:
            continue
        args = [a.decode("utf-8", "replace") for a in raw.split(b"\x00") if a]
        if not args:
            continue
        if f"__node:={name}" in " ".join(args):
            exact.append(pid)
        elif any(os.path.basename(a) == name for a in args):
            exe.append(pid)
        elif name in args:
            token.append(pid)
    return exact or exe or token


def restart_self(delay: float = 0.6) -> None:
    """Restart this web_monitor process so code edits take effect — no terminal,
    no sudo. Re-execs the same Python program (os.execv keeps the PID); the fresh
    interpreter re-imports the package from the symlinked source. If the new code
    fails to boot, the process exits and systemd's Restart=on-failure recovers it.
    Delayed in a background thread so the HTTP response is flushed first."""
    def _do() -> None:
        time.sleep(delay)
        try:
            os.execv(sys.executable, [sys.executable] + sys.argv)
        except Exception:
            os._exit(1)  # hand off to systemd Restart=on-failure
    threading.Thread(target=_do, daemon=True).start()


def shutdown_self(cam=None, delay: float = 0.6) -> None:
    """Cleanly stop the web monitor and STAY stopped, then exit with code 0.
    systemd's Restart=on-failure does NOT restart a clean exit, so the monitor
    stays down until manually started again. Delayed in a background thread so
    the HTTP response is flushed first."""
    def _do() -> None:
        time.sleep(delay)
        os._exit(0)   # clean exit → systemd on-failure will NOT restart
    threading.Thread(target=_do, daemon=True).start()


def kill_node(name: str) -> str:
    """Kill the process(es) hosting `name`. SIGINT first (clean rclpy shutdown),
    escalate to SIGTERM after a grace period. Returns a human-readable result."""
    pids = find_node_pids(name)
    if not pids:
        return f"No killable process found for '{name}' (it may be our own node)."
    for pid in pids:
        try:
            os.kill(pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    time.sleep(1.5)
    escalated = []
    for pid in pids:
        try:
            os.kill(pid, 0)          # still alive?
            os.kill(pid, signal.SIGTERM)
            escalated.append(pid)
        except ProcessLookupError:
            pass
    note = f" (SIGTERM sent to {escalated})" if escalated else ""
    return f"Killed '{name}' → PID(s) {pids}{note}."


def _remote_command(ssh_host: str, command: List[str]) -> List[str]:
    if not ssh_host:
        return command
    return [
        "/usr/bin/ssh",
        "-o", "BatchMode=yes",
        "-o", "ConnectTimeout=2",
        ssh_host,
        *command,
    ]


def systemd_service_status(
    service: str, ssh_host: str = "", user_service: bool = False,
) -> dict:
    """Read one allowlisted systemd unit state without elevated access."""
    if service not in (
        OM6DOF_SERVICE, OM6DOF_PERCEPTION_SERVICE, OM6DOF_DDGNG_SERVICE,
    ):
        raise ValueError(f"unsupported service: {service}")
    result = {
        "active_state": "unknown",
        "sub_state": "unknown",
        "main_pid": 0,
    }
    try:
        systemctl = ["/usr/bin/systemctl"]
        if user_service:
            systemctl.append("--user")
        completed = subprocess.run(
            _remote_command(ssh_host, systemctl + [
                "show",
                service,
                "--property=ActiveState",
                "--property=SubState",
                "--property=MainPID",
                "--no-pager",
            ]),
            capture_output=True,
            text=True,
            timeout=2.0,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return result
    for line in completed.stdout.splitlines():
        key, separator, value = line.partition("=")
        if not separator:
            continue
        if key == "ActiveState":
            result["active_state"] = value.strip() or "unknown"
        elif key == "SubState":
            result["sub_state"] = value.strip() or "unknown"
        elif key == "MainPID":
            try:
                result["main_pid"] = max(0, int(value))
            except ValueError:
                result["main_pid"] = 0
    return result


def om6dof_service_status(ssh_host: str = "") -> dict:
    return systemd_service_status(OM6DOF_SERVICE, ssh_host)


def perception_service_status(ssh_host: str = "") -> dict:
    return systemd_service_status(
        OM6DOF_PERCEPTION_SERVICE, ssh_host, user_service=True
    )


def ddgng_service_status(ssh_host: str = "") -> dict:
    return systemd_service_status(
        OM6DOF_DDGNG_SERVICE, ssh_host, user_service=True
    )


def invoke_om6dof_service_restart(
    ssh_host: str = "",
) -> subprocess.CompletedProcess:
    """Request one exact privileged action; no HTTP value enters this argv."""
    return subprocess.run(
        _remote_command(ssh_host, list(OM6DOF_RESTART_COMMAND)),
        capture_output=True,
        text=True,
        timeout=8.0,
        check=False,
    )


def invoke_perception_service(
    action: str, ssh_host: str = "",
) -> subprocess.CompletedProcess:
    """Start/stop perception using only fixed, sudoers-approved argv."""
    try:
        command = OM6DOF_PERCEPTION_COMMANDS[action]
    except KeyError as exc:
        raise ValueError(f"unsupported perception action: {action}") from exc
    return subprocess.run(
        _remote_command(ssh_host, list(command)),
        capture_output=True,
        text=True,
        timeout=12.0,
        check=False,
    )


def invoke_ddgng_service(
    action: str, ssh_host: str = "",
) -> subprocess.CompletedProcess:
    """Start/stop DD-GNG using only fixed, allowlisted systemctl argv."""
    try:
        command = OM6DOF_DDGNG_COMMANDS[action]
    except KeyError as exc:
        raise ValueError(f"unsupported DD-GNG action: {action}") from exc
    return subprocess.run(
        _remote_command(ssh_host, list(command)),
        capture_output=True,
        text=True,
        timeout=12.0,
        check=False,
    )


def csrf_token_matches(provided: str, expected: str) -> bool:
    """Constant-time comparison that also safely rejects non-ASCII input."""
    try:
        return secrets.compare_digest(
            provided.encode("utf-8"),
            expected.encode("utf-8"),
        )
    except (AttributeError, UnicodeError):
        return False


# --------------------------------------------------------------------------- #
#  ROS node: graph introspection + F3 publisher                               #
# --------------------------------------------------------------------------- #
class MonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("go2w_web_monitor")
        self.flash = ""  # one-shot banner (e.g. kill result), shown then cleared
        self.csrf_token = secrets.token_urlsafe(32)
        self._arm_restart_lock = threading.Lock()
        self._arm_restart_phase = "idle"
        self._arm_restart_message = ""
        self._arm_restart_started = 0.0
        self._controller_state_lock = threading.Lock()
        self._controller_states = {}
        self._controller_states_updated = 0.0
        self._controller_query = None
        self._controller_query_started = 0.0
        self._controller_query_generation = 0
        self.declare_parameter("robot_ssh_host", "")
        self.robot_ssh_host = str(
            self.get_parameter("robot_ssh_host").value
        ).strip()
        self.declare_parameter(
            "camera_topic", "/application_web_monitor/image/compressed"
        )
        self.camera = ForwardedImageStream(
            str(self.get_parameter("camera_topic").value),
        )
        self.declare_parameter(
            "perception_camera_topic",
            "/application_web_monitor/perception/image/compressed",
        )
        self.perception_camera = ForwardedImageStream(
            str(self.get_parameter("perception_camera_topic").value),
        )
        self.declare_parameter(
            "ddgng_camera_topic",
            "/application_web_monitor/ddgng/image/compressed",
        )
        self.ddgng_camera = ForwardedImageStream(
            str(self.get_parameter("ddgng_camera_topic").value),
        )
        self.declare_parameter("audio_topic", "/application/audio/pcm_s16le")
        self.audio = ForwardedPcmStream(
            str(self.get_parameter("audio_topic").value),
        )
        self.declare_parameter(
            "voice_active_topic", "/application/audio/voice_active")
        self.voice_active_topic = str(
            self.get_parameter("voice_active_topic").value)
        self.voice_active = False
        self.audio_source = "unknown"
        self.stt_text = ""
        self.stt_status = "offline"
        self.voice_llm_response = ""
        self.voice_llm_status = "offline"
        self.tts_status = "offline"
        self.tts_speaking = False
        self.perception_tracking_status = "offline"
        self.perception_distance_m = None
        self._pickup_lock = threading.Lock()
        self.pickup_busy = False
        self.pickup_message = "not run yet"
        self.object_tracking_active = False
        self.object_tracking_busy = False
        self.object_tracking_message = "not run yet"
        self.object_search_active = False
        self.object_search_busy = False
        self.object_search_message = "not run yet"
        self.arm_target_active = False
        self.arm_target_state = "idle"
        self.arm_target_mode = ""
        self.arm_target_request_id = ""
        self.arm_target_message = "no target yet"
        self.arm_target_goal = None
        self.arm_target_current = {}
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub_remote = self.create_publisher(
            WirelessController, "/wirelesscontroller", qos
        )
        self.pub_operation_mode = self.create_publisher(
            String, "/om6dof/operation_mode", 10
        )
        self.pub_arm_target = self.create_publisher(
            String, "/om6dof/target_cmd", 10
        )
        self.pub_perception_target = self.create_publisher(
            String, "/om6dof_perception/set_target", 10
        )
        self.pickup_client = self.create_client(
            Trigger, "/run_perception_pick"
        )
        self.pickup_status_client = self.create_client(
            Trigger, "/direct_pick_status"
        )
        self._pickup_status_future = None
        self._pickup_status_future_started = 0.0
        self._pickup_status_generation = 0
        self.object_tracking_start_client = self.create_client(
            Trigger, "/direct_track"
        )
        self.object_tracking_stop_client = self.create_client(
            Trigger, "/direct_stop"
        )
        self.object_search_start_client = self.create_client(
            Trigger, "/direct_search"
        )
        self.object_search_stop_client = self.create_client(
            Trigger, "/direct_search_stop"
        )
        self.object_search_status_client = self.create_client(
            Trigger, "/direct_search_status"
        )
        self._object_search_status_future = None
        self.create_timer(1.0, self._poll_pickup_status)
        self.create_timer(1.0, self._poll_object_search_status)
        # Sport-API publisher for chat-driven motion (Robot Agent mode). Only
        # created when unitree_api is available.
        self.pub_sport = (
            self.create_publisher(SportRequest, SPORT_TOPIC, 10)
            if SportRequest is not None else None
        )
        # Serialise motion: one active move at a time; a new command cancels the
        # previous one. _motion_gen bumps to signal the running loop to quit.
        self._motion_lock = threading.Lock()
        self._motion_gen = 0
        # Controller/teleop state is TRANSIENT_LOCAL, so the dashboard receives
        # the latest value even when it starts after the arm stack.
        self.gripper_state: Optional[str] = None
        grip_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            String, "/om6dof_teleop/gripper_state", self._on_gripper, grip_qos
        )
        self.remote_enabled: Optional[bool] = None
        self.create_subscription(
            Bool,
            "/om6dof/remote_enabled/state",
            self._on_remote_enabled,
            grip_qos,
        )
        self.control_mode: Optional[str] = None
        self.create_subscription(
            String,
            "/om6dof/operation_mode/state",
            self._on_control_mode,
            grip_qos,
        )
        self.create_subscription(
            String,
            "/om6dof/target_status",
            self._on_arm_target_status,
            grip_qos,
        )
        # Battery from /lowstate (~500 Hz). The callback only stores the latest
        # values — no processing — so it stays cheap.
        self.battery_soc: Optional[int] = None
        self.battery_v: Optional[float] = None
        self.battery_a: Optional[float] = None
        self.create_subscription(LowState, "/lowstate", self._on_lowstate, qos)
        self.create_subscription(
            CompressedImage,
            self.camera.topic,
            self.camera.on_image,
            qos,
        )
        self.create_subscription(
            CompressedImage,
            self.perception_camera.topic,
            self.perception_camera.on_image,
            qos,
        )
        self.create_subscription(
            CompressedImage,
            self.ddgng_camera.topic,
            self.ddgng_camera.on_image,
            qos,
        )
        self.create_subscription(
            UInt8MultiArray,
            self.audio.topic,
            self.audio.on_pcm,
            qos,
        )
        self.create_subscription(
            Bool,
            self.voice_active_topic,
            self._on_voice_active,
            grip_qos,
        )
        self.create_subscription(
            String,
            "/application/audio/format",
            self._on_audio_format,
            grip_qos,
        )
        self.create_subscription(
            String,
            "/application/stt/text",
            self._on_stt_text,
            grip_qos,
        )
        self.create_subscription(
            String,
            "/application/stt/status",
            self._on_stt_status,
            grip_qos,
        )
        self.create_subscription(
            String,
            "/application/llm/response",
            self._on_voice_llm_response,
            grip_qos,
        )
        self.create_subscription(
            String,
            "/application/llm/status",
            self._on_voice_llm_status,
            grip_qos,
        )
        self.create_subscription(
            String,
            "/application/tts/status",
            self._on_tts_status,
            grip_qos,
        )
        self.create_subscription(
            String,
            "/om6dof_perception/status",
            self._on_perception_status,
            qos,
        )
        self.create_subscription(
            String,
            "/direct_tracking_status",
            self._on_object_tracking_status,
            grip_qos,
        )
        self.create_subscription(
            Bool,
            "/application/tts/speaking",
            self._on_tts_speaking,
            grip_qos,
        )
        self.controller_list_client = None
        if ListControllers is not None:
            self.controller_list_client = self.create_client(
                ListControllers,
                "/controller_manager/list_controllers",
            )
            self.create_timer(1.0, self._poll_controller_states)
        else:
            self.get_logger().warn(
                "controller_manager_msgs unavailable; controller health "
                "details are disabled")

    def _on_gripper(self, msg: String) -> None:
        self.gripper_state = msg.data

    def _on_remote_enabled(self, msg: Bool) -> None:
        self.remote_enabled = bool(msg.data)

    def _on_control_mode(self, msg: String) -> None:
        self.control_mode = msg.data.strip().upper()

    def _on_arm_target_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
            if not isinstance(status, dict):
                raise ValueError("target status must be an object")
            state = str(status.get("state", "unknown")).strip().lower()
            allowed_states = {
                "idle", "requesting", "running", "reached", "rejected",
                "stopping", "stopped", "timeout", "blocked",
            }
            if state not in allowed_states:
                state = "unknown"
            mode = str(status.get("mode", "")).strip().upper()
            if mode not in ("JOINT", "CARTESIAN", "CYLINDRICAL"):
                mode = ""
            current = status.get("current", {})
            if not isinstance(current, dict):
                current = {}
            clean_current = {}
            for key in ("joint", "cartesian", "cylindrical"):
                try:
                    values = [float(value) for value in current.get(key, ())]
                except (TypeError, ValueError):
                    continue
                if len(values) == 6 and all(math.isfinite(value) for value in values):
                    clean_current[key] = values
            goal = status.get("goal")
            if goal is not None:
                try:
                    goal = [float(value) for value in goal]
                except (TypeError, ValueError):
                    goal = None
                if goal is not None and (
                    len(goal) != 6
                    or not all(math.isfinite(value) for value in goal)
                ):
                    goal = None
            self.arm_target_active = bool(status.get("active", state == "running"))
            self.arm_target_state = state
            self.arm_target_mode = mode
            self.arm_target_request_id = str(status.get("request_id", ""))[:80]
            self.arm_target_message = (
                "no target yet"
                if state == "idle"
                else str(status.get(
                    "message", "target status has no message"
                ))[:500]
            )
            self.arm_target_goal = goal
            self.arm_target_current = clean_current
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warn("Ignoring malformed /om6dof/target_status")

    def _on_voice_active(self, msg: Bool) -> None:
        self.voice_active = bool(msg.data)

    def _on_audio_format(self, msg: String) -> None:
        try:
            source = str(json.loads(msg.data).get("source", "")).strip()
        except (AttributeError, TypeError, ValueError, json.JSONDecodeError):
            source = ""
        self.audio_source = {
            "dji_wireless_mic_rx": "DJI Wireless Mic Rx",
            "unitree_builtin": "Unitree built-in microphone",
        }.get(source, source or "Unitree built-in microphone")

    def _on_stt_text(self, msg: String) -> None:
        self.stt_text = msg.data.strip()

    def _on_stt_status(self, msg: String) -> None:
        self.stt_status = msg.data.strip() or "unknown"

    def _on_voice_llm_response(self, msg: String) -> None:
        self.voice_llm_response = msg.data.strip()

    def _on_voice_llm_status(self, msg: String) -> None:
        self.voice_llm_status = msg.data.strip() or "unknown"

    def _on_tts_status(self, msg: String) -> None:
        self.tts_status = msg.data.strip() or "unknown"

    def _on_tts_speaking(self, msg: Bool) -> None:
        self.tts_speaking = bool(msg.data)

    def _on_perception_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
            target_state = str(status.get("target", {}).get("state", "unknown"))
            ee_state = str(status.get("ee", {}).get("state", "unknown"))
            self.perception_tracking_status = (
                f"target={target_state}, EoE={ee_state}"
            )
            distance = status.get("distance_m")
            distance = float(distance) if distance is not None else None
            self.perception_distance_m = (
                distance if distance is not None and math.isfinite(distance)
                else None
            )
        except (TypeError, ValueError, json.JSONDecodeError):
            self.perception_tracking_status = "invalid status"
            self.perception_distance_m = None

    def _on_lowstate(self, msg: LowState) -> None:
        try:
            self.battery_soc = int(msg.bms_state.soc)
            self.battery_v = float(msg.power_v)
            self.battery_a = float(msg.power_a)
        except Exception:
            pass

    def nodes(self) -> List[Tuple[str, str]]:
        """(name, namespace) tuples, unsorted — duplicates preserved."""
        return list(self.get_node_names_and_namespaces())

    def topics(self) -> List[Tuple[str, List[str]]]:
        return sorted(self.get_topic_names_and_types(), key=lambda x: x[0])

    def teleop_running(self) -> bool:
        return self.remote_enabled is True

    def tap_f3(self) -> None:
        """Publish an F3 edge so ros2_control switches arm ownership."""
        m = WirelessController()
        for _ in range(5):
            m.keys = BTN_F3
            self.pub_remote.publish(m)
            time.sleep(0.05)
        m.keys = 0
        self.pub_remote.publish(m)

    def set_operation_mode(self, mode: str) -> None:
        self.pub_operation_mode.publish(String(data=mode.strip().upper()))

    def request_arm_target(
        self, mode: str, values: List[float]
    ) -> Tuple[bool, str]:
        normalized = str(mode).strip().upper()
        if normalized not in ("JOINT", "CARTESIAN", "CYLINDRICAL"):
            return False, "Target rejected: invalid mode."
        try:
            target = [float(value) for value in values]
        except (TypeError, ValueError):
            return False, "Target rejected: all values must be numbers."
        if len(target) != 6 or not all(math.isfinite(value) for value in target):
            return False, "Target rejected: exactly six finite numbers are required."
        with self._pickup_lock:
            if self.remote_enabled is not True:
                return False, (
                    "Target rejected: enable remote arm control and wait for "
                    "READY to finish."
                )
            if self.control_mode not in (
                "JOINT", "CARTESIAN", "CYLINDRICAL"
            ):
                return False, (
                    "Target rejected: the controller has not finished READY."
                )
            if self.pickup_busy:
                return False, "Target rejected: pickup is running."
            if self.object_tracking_active or self.object_tracking_busy:
                return False, "Target rejected: tracking is running."
            if self.object_search_active or self.object_search_busy:
                return False, "Target rejected: search is running."
            if self._arm_restart_phase == "restarting":
                return False, "Target rejected: the OM6DOF stack is restarting."
            if self.arm_target_active:
                return False, (
                    "Target rejected: another target is still running; press "
                    "Stop target."
                )
            request_id = secrets.token_hex(8)
            payload = json.dumps({
                "action": "move",
                "mode": normalized,
                "values": target,
                "request_id": request_id,
            }, separators=(",", ":"), allow_nan=False)
            self.arm_target_active = True
            self.arm_target_state = "requesting"
            self.arm_target_mode = normalized
            self.arm_target_request_id = request_id
            self.arm_target_goal = target
            self.arm_target_message = f"Sending {normalized} target…"
        try:
            self.pub_arm_target.publish(String(data=payload))
        except Exception as exc:
            with self._pickup_lock:
                self.arm_target_active = False
                self.arm_target_state = "rejected"
                self.arm_target_message = f"Failed to send target: {exc}"
            return False, self.arm_target_message
        return True, (
            f"{normalized} target sent; wait for the controller status."
        )

    def request_arm_target_stop(self) -> Tuple[bool, str]:
        request_id = secrets.token_hex(8)
        payload = json.dumps({
            "action": "stop", "request_id": request_id,
        }, separators=(",", ":"))
        with self._pickup_lock:
            was_active = self.arm_target_active
            self.arm_target_state = "stopping"
            self.arm_target_request_id = request_id
            self.arm_target_message = "Sending Stop target…"
        try:
            self.pub_arm_target.publish(String(data=payload))
        except Exception as exc:
            with self._pickup_lock:
                self.arm_target_state = "blocked" if was_active else "rejected"
                self.arm_target_message = f"Failed to send Stop target: {exc}"
            return False, self.arm_target_message
        return True, "Stop target sent; the controller will hold the arm position."

    def set_perception_target(self, description: str) -> None:
        self.pub_perception_target.publish(String(data=description.strip()))

    def _on_object_tracking_status(self, msg: String) -> None:
        text = msg.data.strip()
        with self._pickup_lock:
            self.object_tracking_active = text.startswith("active:")
            self.object_tracking_message = text or "empty status"

    def request_object_tracking(self, enable: bool) -> Tuple[bool, str]:
        with self._pickup_lock:
            if self.object_tracking_busy:
                return False, "The tracking command is still being processed."
            if enable and self.pickup_busy:
                return False, "Pickup is running."
            if enable and self.object_search_active:
                return False, "Search is running."
            if enable and self.remote_enabled is True:
                return False, "Disable remote arm control (F3) before tracking."
            if enable and self.perception_distance_m is None:
                return False, "No perception object has been detected yet."
            client = (self.object_tracking_start_client if enable
                      else self.object_tracking_stop_client)
            if not client.service_is_ready():
                return False, "The tracking backend is not ready."
            self.object_tracking_busy = True
            self.object_tracking_message = (
                "Starting pan–tilt tracking…" if enable
                else "Stopping tracking…"
            )
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda result: self._on_object_tracking_response(result, enable))
        return True, self.object_tracking_message

    def _on_object_tracking_response(self, future, requested_enable: bool) -> None:
        try:
            response = future.result()
            success = bool(response.success)
            message = response.message or (
                "Tracking active." if requested_enable else "Tracking stopped."
            )
        except Exception as exc:
            success = False
            message = f"Tracking service failed: {exc}"
        with self._pickup_lock:
            self.object_tracking_busy = False
            if success:
                self.object_tracking_active = requested_enable
            self.object_tracking_message = message

    def request_object_search(self, enable: bool) -> Tuple[bool, str]:
        with self._pickup_lock:
            if self.object_search_busy:
                return False, "The search command is still being processed."
            if enable and (self.pickup_busy or self.object_tracking_active):
                return False, "Stop pickup/tracking before starting the search."
            if enable and self.remote_enabled is True:
                return False, "Disable remote arm control (F3) before searching."
            client = (self.object_search_start_client if enable
                      else self.object_search_stop_client)
            if not client.service_is_ready():
                return False, "The search backend is not ready."
            self.object_search_busy = True
            self.object_search_message = (
                "Starting low → medium → high sweep…" if enable
                else "Stopping search…"
            )
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda result: self._on_object_search_response(result, enable))
        return True, self.object_search_message

    def _on_object_search_response(self, future, requested_enable: bool) -> None:
        try:
            response = future.result()
            success = bool(response.success)
            message = response.message or (
                "Search active." if requested_enable
                else "Search stopped."
            )
        except Exception as exc:
            success = False
            message = f"Search service failed: {exc}"
        with self._pickup_lock:
            self.object_search_busy = False
            if success:
                self.object_search_active = requested_enable
            self.object_search_message = message

    def _poll_object_search_status(self) -> None:
        future = self._object_search_status_future
        if future is not None and not future.done():
            return
        if not self.object_search_status_client.service_is_ready():
            return
        self._object_search_status_future = (
            self.object_search_status_client.call_async(Trigger.Request()))
        self._object_search_status_future.add_done_callback(
            self._on_object_search_status_response)

    def _on_object_search_status_response(self, future) -> None:
        try:
            response = future.result()
            active = bool(response.success)
            message = response.message.strip()
        except Exception:
            return
        with self._pickup_lock:
            self.object_search_active = active
            if message:
                self.object_search_message = message

    def request_perception_pick(self) -> Tuple[bool, str]:
        with self._pickup_lock:
            if self.pickup_busy:
                return False, "Pickup is already running."
            if self.object_search_active:
                return False, "Search is running."
            if self.remote_enabled is True:
                return False, "Disable remote arm control (F3) before pickup."
            if self.perception_distance_m is None:
                return False, "The target or EoE distance is not valid yet."
            if not self.pickup_client.service_is_ready():
                return False, "The pickup backend is not ready; restart perception."
            self.pickup_busy = True
            self.pickup_message = "Pickup request sent; waiting for the backend."
        future = self.pickup_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_pickup_response)
        return True, self.pickup_message

    def _poll_pickup_status(self) -> None:
        now = time.monotonic()
        future = self._pickup_status_future
        if future is not None and not future.done():
            if now - self._pickup_status_future_started \
                    < PICKUP_STATUS_TIMEOUT_S:
                return
            # A ROS service Future can remain pending forever when its server
            # restarts between discovery and response. Remove it and issue a
            # fresh request; otherwise pickup_busy remains frozen in the GUI.
            try:
                self.pickup_status_client.remove_pending_request(future)
            except Exception:
                pass
            self._pickup_status_generation += 1
            self._pickup_status_future = None
            self.get_logger().warn(
                "Pickup status request timed out; retrying with a fresh request.")
        if not self.pickup_status_client.service_is_ready():
            return
        self._pickup_status_generation += 1
        generation = self._pickup_status_generation
        self._pickup_status_future = self.pickup_status_client.call_async(
            Trigger.Request())
        self._pickup_status_future_started = now
        self._pickup_status_future.add_done_callback(
            lambda completed: self._on_pickup_status_response(
                completed, generation))

    def _on_pickup_status_response(self, future, generation: int) -> None:
        if generation != self._pickup_status_generation:
            return
        try:
            response = future.result()
            pickup_active = bool(response.success)
            message = response.message.strip()
        except Exception:
            return
        if not message:
            return
        non_pick_status = any(text in message.lower() for text in (
            "tracking pan-tilt", "searching low", "searching medium",
            "searching high", "search has not run",
        ))
        with self._pickup_lock:
            self.pickup_busy = pickup_active
            if pickup_active or not non_pick_status:
                self.pickup_message = message

    def _on_pickup_response(self, future) -> None:
        try:
            response = future.result()
            success = bool(response.success)
            message = response.message or (
                "Pickup started." if success else "Pickup rejected."
            )
        except Exception as exc:
            success = False
            message = f"Pickup service failed: {exc}"
        with self._pickup_lock:
            self.pickup_busy = success
            self.pickup_message = message

    def arm_stack_missing_nodes(self) -> List[str]:
        try:
            names = {name for name, _namespace in self.nodes()}
        except Exception:
            return sorted(OM6DOF_REQUIRED_NODES)
        return sorted(OM6DOF_REQUIRED_NODES - names)

    def _poll_controller_states(self) -> None:
        if self.controller_list_client is None or ListControllers is None:
            return
        now = time.monotonic()
        expired_future = None
        with self._controller_state_lock:
            if self._controller_query is not None:
                if (
                    now - self._controller_query_started
                    <= CONTROLLER_QUERY_TIMEOUT
                ):
                    return
                expired_future = self._controller_query
                self._controller_query_generation += 1
                self._controller_query = None
                self._controller_query_started = 0.0
            generation = self._controller_query_generation
        if expired_future is not None:
            try:
                self.controller_list_client.remove_pending_request(expired_future)
            except Exception:
                pass
            self.get_logger().warn(
                "list_controllers timed out; retrying controller health query"
            )
        if not self.controller_list_client.service_is_ready():
            return
        future = self.controller_list_client.call_async(ListControllers.Request())
        discard_future = False
        with self._controller_state_lock:
            if generation != self._controller_query_generation:
                discard_future = True
            else:
                self._controller_query = future
                self._controller_query_started = time.monotonic()
        if discard_future:
            try:
                self.controller_list_client.remove_pending_request(future)
            except Exception:
                pass
            return
        future.add_done_callback(
            lambda completed: self._on_controller_states(completed, generation)
        )

    def _on_controller_states(self, future, generation: int) -> None:
        with self._controller_state_lock:
            if (
                generation != self._controller_query_generation
                or self._controller_query is not future
            ):
                return
            self._controller_query = None
            self._controller_query_started = 0.0
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"list_controllers failed: {exc}")
            return
        states = {
            controller.name: controller.state
            for controller in response.controller
        }
        with self._controller_state_lock:
            if generation != self._controller_query_generation:
                return
            self._controller_states = states
            self._controller_states_updated = time.monotonic()

    def _invalidate_controller_states(self) -> None:
        """Discard pre-restart service replies and wait for the new manager."""
        pending_future = None
        with self._controller_state_lock:
            pending_future = self._controller_query
            self._controller_query_generation += 1
            self._controller_query = None
            self._controller_query_started = 0.0
            self._controller_states = {}
            self._controller_states_updated = 0.0
        if pending_future is not None and self.controller_list_client is not None:
            try:
                self.controller_list_client.remove_pending_request(pending_future)
            except Exception:
                pass

    def controller_state_snapshot(self) -> Tuple[dict, float]:
        with self._controller_state_lock:
            return dict(self._controller_states), self._controller_states_updated

    def arm_controller_issues(self, max_age: float = 3.0) -> List[str]:
        if self.controller_list_client is None:
            return ["controller health unavailable on application host"]
        states, updated = self.controller_state_snapshot()
        if updated <= 0.0 or time.monotonic() - updated > max_age:
            return ["controller states unavailable"]

        issues = []
        for name in OM6DOF_ALWAYS_ACTIVE_CONTROLLERS:
            if states.get(name) != "active":
                issues.append(f"{name}={states.get(name, 'missing')}")

        arm_state = states.get(OM6DOF_ARM_CONTROLLERS[0], "missing")
        forward_state = states.get(OM6DOF_ARM_CONTROLLERS[1], "missing")
        valid_owner_pair = (
            (arm_state == "active" and forward_state == "inactive")
            or (arm_state == "inactive" and forward_state == "active")
        )
        if not valid_owner_pair:
            issues.append(
                f"arm_controller={arm_state}, "
                f"forward_position_controller={forward_state}"
            )
        return issues

    def arm_restart_snapshot(self) -> dict:
        with self._arm_restart_lock:
            return {
                "phase": self._arm_restart_phase,
                "message": self._arm_restart_message,
                "started": self._arm_restart_started,
            }

    def _set_arm_restart_state(self, phase: str, message: str) -> None:
        with self._arm_restart_lock:
            self._arm_restart_phase = phase
            self._arm_restart_message = message

    def request_arm_stack_restart(self) -> Tuple[bool, str]:
        with self._arm_restart_lock:
            if self._arm_restart_phase == "restarting":
                return False, "OM6DOF stack restart is already in progress."
            self._arm_restart_phase = "restarting"
            self._arm_restart_message = (
                "Restart requested; waiting for systemd and ROS nodes."
            )
            self._arm_restart_started = time.monotonic()
        threading.Thread(
            target=self._restart_arm_stack_worker,
            name="om6dof-service-restart",
            daemon=True,
        ).start()
        return True, "OM6DOF stack restart requested."

    def _restart_arm_stack_worker(self) -> None:
        ssh_host = getattr(self, "robot_ssh_host", "")
        old_status = (
            om6dof_service_status(ssh_host)
            if ssh_host else om6dof_service_status()
        )
        old_pid = int(old_status["main_pid"])
        try:
            completed = (
                invoke_om6dof_service_restart(ssh_host)
                if ssh_host else invoke_om6dof_service_restart()
            )
        except subprocess.TimeoutExpired:
            message = "OM6DOF restart command timed out."
            self._set_arm_restart_state("failed", message)
            self.get_logger().error(message)
            return
        except OSError as exc:
            message = f"OM6DOF restart command failed: {exc}"
            self._set_arm_restart_state("failed", message)
            self.get_logger().error(message)
            return

        if completed.returncode != 0:
            detail = (completed.stderr or completed.stdout).strip()
            if len(detail) > 300:
                detail = detail[:297] + "..."
            message = "OM6DOF restart denied"
            if detail:
                message += f": {detail}"
            message += ". Install the scoped web-monitor sudoers rule."
            self._set_arm_restart_state("failed", message)
            self.get_logger().error(message)
            return

        deadline = time.monotonic() + 45.0
        last_status = old_status
        last_missing = self.arm_stack_missing_nodes()
        last_controller_issues = self.arm_controller_issues()
        observed_new_pid = 0
        while time.monotonic() < deadline:
            last_status = (
                om6dof_service_status(ssh_host)
                if ssh_host else om6dof_service_status()
            )
            last_missing = self.arm_stack_missing_nodes()
            new_pid = int(last_status["main_pid"])
            pid_replaced = new_pid > 0 and (old_pid <= 0 or new_pid != old_pid)
            if pid_replaced and new_pid != observed_new_pid:
                # Any cached response can belong to the previous manager. A
                # fresh list_controllers response is required after each new
                # systemd MainPID is observed.
                self._invalidate_controller_states()
                observed_new_pid = new_pid
            last_controller_issues = self.arm_controller_issues()
            if (
                last_status["active_state"] == "active"
                and pid_replaced
                and not last_missing
                and not last_controller_issues
            ):
                message = (
                    f"OM6DOF stack READY; MainPID {old_pid or '-'} -> "
                    f"{new_pid}, controller_manager and controller recovered."
                )
                self._set_arm_restart_state("ready", message)
                self.get_logger().info(message)
                return
            time.sleep(0.5)

        missing = ", ".join(last_missing) if last_missing else "none"
        controller_issues = (
            "; ".join(last_controller_issues)
            if last_controller_issues else "none"
        )
        message = (
            "OM6DOF restart verification timed out: "
            f"service={last_status['active_state']}/"
            f"{last_status['sub_state']}, missing nodes={missing}, "
            f"controller issues={controller_issues}."
        )
        self._set_arm_restart_state("failed", message)
        self.get_logger().error(message)

    # ----------------------------------------------------------------------- #
    #  Sport-mode motion (chat "Robot Agent" mode)                            #
    # ----------------------------------------------------------------------- #
    def _sport(self, api_id: int, parameter: str = "") -> None:
        if self.pub_sport is None:
            raise RuntimeError(
                "unitree_api not available — cannot drive the robot from chat."
            )
        req = SportRequest()
        req.header.identity.api_id = api_id
        req.parameter = parameter
        self.pub_sport.publish(req)

    def sport_stop(self) -> None:
        self._motion_gen += 1  # cancel any running move loop
        self._sport(API_STOP_MOVE)

    def sport_stand(self) -> None:
        self._motion_gen += 1
        self._sport(API_STAND_UP)
        self._sport(API_BALANCE_STAND)

    def sport_gait(self, code: int) -> None:
        """Switch locomotion mode (api_id 1011). Sent a few times because the sport
        controller can drop a single SwitchGait (same repeat trick as the demo and
        go2w_cmd_vel_control)."""
        param = '{"data":%d}' % int(code)
        for _ in range(3):
            self._sport(API_SWITCH_GAIT, param)
            time.sleep(0.1)

    def sport_speed(self, level: int) -> None:
        """Set speed gear via SpeedLevel (api_id 1015, -1/0/1). Per the Unitree
        doc this only takes effect in the default gait, so for any non-normal gear
        we drop back to gait 0 first."""
        level = max(-1, min(1, int(level)))
        if level != 0:
            self._sport(API_SWITCH_GAIT, '{"data":0}')
            time.sleep(0.1)
        for _ in range(3):
            self._sport(API_SPEED_LEVEL, '{"data":%d}' % level)
            time.sleep(0.1)

    def sport_damp(self) -> None:
        self._motion_gen += 1
        self._sport(API_DAMP)

    def sport_lie_down(self) -> None:
        """Lie down. StandDown only responds when the robot is in the standing-lock
        (StandUp) or damping state — from balance-stand or a gait it is ignored. So
        stop, enter StandUp lock, wait for it to settle, then StandDown. Runs in a
        thread because of the settle delay."""
        with self._motion_lock:
            self._motion_gen += 1
            gen = self._motion_gen

        def _run() -> None:
            self._sport(API_STOP_MOVE)
            time.sleep(0.2)
            self._sport(API_STAND_UP)          # required precondition (joint lock)
            for _ in range(5):                 # wait ~0.5 s for stand-up to settle
                if gen != self._motion_gen:    # superseded by a newer command
                    return
                time.sleep(0.1)
            self._sport(API_STAND_DOWN)

        threading.Thread(target=_run, daemon=True).start()

    def sport_recover(self) -> None:
        self._motion_gen += 1
        self._sport(API_RECOVERY_STAND)

    def sport_move(self, vx: float, vy: float, wz: float, duration: float) -> None:
        """Stream Move at 20 Hz for `duration` s (clamped), then StopMove. Runs in
        a daemon thread so the HTTP handler returns immediately."""
        vx = max(-MAX_VX, min(MAX_VX, float(vx)))
        vy = max(-MAX_VY, min(MAX_VY, float(vy)))
        wz = max(-MAX_WZ, min(MAX_WZ, float(wz)))
        duration = max(0.0, min(MAX_DURATION, float(duration)))
        with self._motion_lock:
            self._motion_gen += 1
            gen = self._motion_gen

        def _run() -> None:
            param = '{"x":%.4f,"y":%.4f,"z":%.4f}' % (vx, vy, wz)
            end = time.time() + duration
            try:
                while time.time() < end and gen == self._motion_gen:
                    self._sport(API_MOVE, param)
                    time.sleep(0.05)
            finally:
                if gen == self._motion_gen:  # only stop if not superseded
                    self._sport(API_STOP_MOVE)

        threading.Thread(target=_run, daemon=True).start()


# --------------------------------------------------------------------------- #
#  Camera: forwarded ROS CompressedImage → shared latest JPEG                 #
# --------------------------------------------------------------------------- #
class ForwardedImageStream:
    ACTIVE_TIMEOUT_S = 4.0

    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        self.latest: Optional[bytes] = None
        self.last_frame = 0.0
        self.frame_seq = 0

    def on_image(self, msg: CompressedImage) -> None:
        frame = bytes(msg.data)
        if not frame:
            return
        with self.cond:
            self.latest = frame
            self.last_frame = time.monotonic()
            self.frame_seq += 1
            self.cond.notify_all()

    def available(self) -> bool:
        with self.lock:
            return (
                self.latest is not None
                and time.monotonic() - self.last_frame < self.ACTIVE_TIMEOUT_S
            )

    def next_frame(self, last_seq: int, timeout: float = 2.0):
        with self.cond:
            self.cond.wait_for(
                lambda: self.frame_seq != last_seq,
                timeout=timeout,
            )
            return self.latest, self.frame_seq

    def snapshot(self) -> Optional[bytes]:
        """Return a copy of the latest recent JPEG for one-shot VLM input."""
        with self.lock:
            if (self.latest is None
                    or time.monotonic() - self.last_frame
                    >= self.ACTIVE_TIMEOUT_S):
                return None
            return bytes(self.latest)


class ForwardedPcmStream:
    """Thread-safe latest-frame stream for mono 48 kHz s16le PCM."""

    ACTIVE_TIMEOUT_S = 2.0
    SAMPLE_RATE = 48000
    CHANNELS = 1

    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        self.latest: Optional[bytes] = None
        self.last_frame = 0.0
        self.frame_seq = 0

    def on_pcm(self, msg: UInt8MultiArray) -> None:
        pcm = bytes(msg.data)
        if not pcm or len(pcm) % 2:
            return
        with self.cond:
            self.latest = pcm
            self.last_frame = time.monotonic()
            self.frame_seq += 1
            self.cond.notify_all()

    def available(self) -> bool:
        with self.lock:
            return (
                self.latest is not None
                and time.monotonic() - self.last_frame < self.ACTIVE_TIMEOUT_S
            )

    def next_chunk(self, last_seq: int, timeout: float = 1.0):
        with self.cond:
            self.cond.wait_for(
                lambda: self.frame_seq != last_seq,
                timeout=timeout,
            )
            return self.latest, self.frame_seq


# --------------------------------------------------------------------------- #
#  System info helpers (best-effort, never raise)                             #
# --------------------------------------------------------------------------- #
def _read(path: str) -> str:
    try:
        with open(path) as f:
            return f.read()
    except Exception:
        return ""


def sys_info() -> dict:
    info = {}
    info["hostname"] = socket.gethostname()
    try:
        info["ips"] = subprocess.run(
            ["hostname", "-I"], capture_output=True, text=True, timeout=2
        ).stdout.strip()
    except Exception:
        info["ips"] = "?"
    up = _read("/proc/uptime").split()
    if up:
        secs = int(float(up[0]))
        info["uptime"] = f"{secs // 3600}h {(secs % 3600) // 60}m"
    else:
        info["uptime"] = "?"
    try:
        info["load"] = ", ".join(f"{x:.2f}" for x in os.getloadavg())
    except Exception:
        info["load"] = "?"
    mem = {}
    for line in _read("/proc/meminfo").splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0] in ("MemTotal:", "MemAvailable:"):
            mem[parts[0]] = int(parts[1])
    if "MemTotal:" in mem and "MemAvailable:" in mem:
        used_kb = mem["MemTotal:"] - mem["MemAvailable:"]
        total_kb = mem["MemTotal:"]
        used = used_kb / 1e6
        total = total_kb / 1e6
        info["ram"] = f"{used:.1f} / {total:.1f} GB"
        info["ram_percent"] = max(
            0, min(100, round(used_kb * 100 / total_kb))
        )
    else:
        info["ram"] = "?"
        info["ram_percent"] = None
    # CPU temp: highest thermal zone reading
    temps = []
    for zone in range(12):
        raw = _read(f"/sys/class/thermal/thermal_zone{zone}/temp").strip()
        if raw.isdigit():
            temps.append(int(raw) / 1000.0)
    info["temp"] = f"{max(temps):.1f} °C" if temps else "?"
    info["domain_id"] = os.environ.get("ROS_DOMAIN_ID", "0 (default)")
    info["arm_bus"] = os.path.exists(ARM_BUS_DEVICE)
    return info


# --------------------------------------------------------------------------- #
#  HTML rendering                                                             #
# --------------------------------------------------------------------------- #
# --------------------------------------------------------------------------- #
#  AI chat backends: local Ollama, or the codex / claude CLIs                 #
# --------------------------------------------------------------------------- #
OLLAMA_URL = "http://localhost:11434"
CHAT_SYSTEM = ("You are a concise assistant embedded in a Go2W quadruped "
               "robot's web dashboard. Answer briefly.")


def list_ollama_models() -> List[str]:
    try:
        with urllib.request.urlopen(OLLAMA_URL + "/api/tags", timeout=3) as r:
            data = json.loads(r.read().decode("utf-8"))
        return [m["name"] for m in data.get("models", [])]
    except Exception:
        return []


def model_param_billions(name: str) -> float:
    """Parse the parameter count in billions from a model name ('qwen2.5:7b'→7.0,
    'llama3.2:3b'→3.0). Used to order/choose models smallest-first (smaller = more
    likely to fit the Orin GPU). Unknown → large, so it sorts last."""
    m = re.search(r"(\d+(?:\.\d+)?)\s*b\b", name.lower())
    return float(m.group(1)) if m else 999.0


def list_running_ollama() -> List[str]:
    """Models Ollama currently holds in memory (like `ollama ps`). These are the
    ones consuming GPU/RAM and worth killing."""
    try:
        with urllib.request.urlopen(OLLAMA_URL + "/api/ps", timeout=3) as r:
            data = json.loads(r.read().decode("utf-8"))
        return [m["name"] for m in data.get("models", [])]
    except Exception:
        return []


def stop_ollama(model: str) -> Tuple[bool, str]:
    """Unload a loaded model from memory (frees GPU/RAM) via keep_alive=0 — the
    documented Ollama way to stop a running model."""
    payload = json.dumps({"model": model, "keep_alive": 0}).encode("utf-8")
    req = urllib.request.Request(
        OLLAMA_URL + "/api/generate", data=payload,
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            r.read()
        return True, model
    except Exception as exc:
        return False, f"{model}: {exc}"


def preload_ollama(model: str) -> Tuple[bool, str]:
    """Load a model into memory without generating anything (Ollama loads on an
    empty prompt), so the first real chat is fast. keep_alive keeps it resident."""
    payload = json.dumps({"model": model, "keep_alive": "30m"}).encode("utf-8")
    req = urllib.request.Request(
        OLLAMA_URL + "/api/generate", data=payload,
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=180) as r:
            r.read()
        return True, model
    except Exception as exc:
        return False, f"{model}: {exc}"


def raw_ollama_model(value: str) -> Optional[str]:
    """Turn a dropdown value ('agent:qwen2.5:7b' / 'ollama:llama3.2:3b') into the
    bare Ollama model name. Returns None for non-Ollama backends (codex/claude)."""
    if ":" not in value:
        return None
    prefix, rest = value.split(":", 1)
    return rest if prefix in ("agent", "ollama") else None


def stop_running_ollama() -> str:
    """Unload every currently-loaded model. Returns a human-readable result."""
    running = list_running_ollama()
    if not running:
        return "No LLM is loaded — nothing to kill."
    ok, failed = [], []
    for m in running:
        good, info = stop_ollama(m)
        (ok if good else failed).append(info)
    parts = []
    if ok:
        parts.append(f"Killed: {', '.join(ok)}")
    if failed:
        parts.append(f"failed: {', '.join(failed)}")
    return " · ".join(parts)


def chat_ollama(model: str, message: str) -> Tuple[Optional[str], Optional[str]]:
    payload = json.dumps({
        "model": model, "prompt": message,
        "system": CHAT_SYSTEM, "stream": False,
        # Keep the KV cache small so the model fits in the Orin's shared GPU
        # memory (default 32k context OOMs on this rig).
        "options": {"num_ctx": 2048, "num_predict": 256},
    }).encode("utf-8")
    req = urllib.request.Request(
        OLLAMA_URL + "/api/generate", data=payload,
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=180) as r:
            data = json.loads(r.read().decode("utf-8"))
        return (data.get("response", "").strip() or "(empty response)"), None
    except Exception as exc:
        return None, f"ollama error: {exc}"


def chat_cli(argv: List[str], message: str) -> Tuple[Optional[str], Optional[str]]:
    """Run an LLM CLI (codex / claude) with the message as a trailing arg.
    No shell — message is passed as a single argv element."""
    exe = argv[0]
    if shutil.which(exe) is None:
        return None, (f"`{exe}` CLI is not installed on this machine. "
                      f"Install it, then this option will work.")
    try:
        p = subprocess.run(
            argv + [message], capture_output=True, text=True, timeout=300,
        )
    except subprocess.TimeoutExpired:
        return None, f"{exe} timed out (300 s)."
    except Exception as exc:
        return None, f"{exe} failed: {exc}"
    out = (p.stdout or "").strip()
    if not out:
        out = (p.stderr or "").strip() or f"({exe} returned no output, rc={p.returncode})"
    return out, None


def route_chat(model: str, message: str) -> Tuple[Optional[str], Optional[str]]:
    if model.startswith("ollama:"):
        return chat_ollama(model.split(":", 1)[1], message)
    if model == "codex":
        # `codex exec <prompt>` = non-interactive one-shot (adjust if your CLI differs)
        return chat_cli(["codex", "exec"], message)
    if model == "claude":
        # `claude -p <prompt>` = headless print mode
        return chat_cli(["claude", "-p"], message)
    return None, f"unknown model '{model}'"


# --------------------------------------------------------------------------- #
#  Robot Agent: local LLM → JSON action → robot motion                        #
# --------------------------------------------------------------------------- #
# Minimal fallback used only if the skill .md can't be found on disk. The
# authoritative, editable version lives in skills/go2w_control_skill.md.
_FALLBACK_SKILL = (
    "You control a Unitree Go2W robot. Reply with ONE JSON object only, no prose. "
    'Schema: {"action":"move|stop|stand|teleop|say", "vx":f,"vy":f,"wz":f,'
    '"duration":f, "enable":bool, "reply":"short confirmation"}. '
    "move: vx forward+/back- (<=0.4 m/s), vy left+/right- (<=0.3), wz left+/right- "
    "(<=0.8 rad/s), duration seconds (<=8). Forward distance: duration=metres/0.3. "
    "Turn: duration=radians/0.6 (90deg=1.57rad). Reply in the user's language."
)


def load_skill() -> str:
    """Return the Robot Agent system prompt. Prefer the installed skill .md
    (share/application_web_monitor/skills/), extracting the text between the
    <<<SKILL and
    SKILL>>> markers; fall back to the embedded minimal prompt."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(
            get_package_share_directory("application_web_monitor"),
            "skills", "go2w_control_skill.md",
        )
        with open(path, encoding="utf-8") as f:
            text = f.read()
        # Match the markers only where they stand alone on their own line, so a
        # prose mention of "<<<SKILL"/"SKILL>>>" elsewhere in the doc is ignored.
        marker_a, marker_b = "<<<SKILL\n", "\nSKILL>>>"
        start = text.find(marker_a)
        end = text.find(marker_b)
        if start != -1 and end != -1:
            return text[start + len(marker_a):end].strip()
    except Exception:
        pass
    return _FALLBACK_SKILL


def extract_action(reply: str) -> Optional[dict]:
    """Pull the first JSON object out of an LLM reply (tolerates code fences and
    stray prose around it). Returns the parsed dict, or None if none/invalid."""
    if not reply:
        return None
    depth = 0
    start = -1
    for i, ch in enumerate(reply):
        if ch == "{":
            if depth == 0:
                start = i
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0 and start != -1:
                try:
                    obj = json.loads(reply[start:i + 1])
                    if isinstance(obj, dict) and "action" in obj:
                        return obj
                except Exception:
                    start = -1  # not valid JSON; keep scanning
    return None


def execute_action(node: "MonitorNode", act: dict) -> Tuple[Optional[str], Optional[str]]:
    """Perform the parsed action on the robot. Returns (reply, error)."""
    action = str(act.get("action", "")).lower()
    reply = str(act.get("reply", "")).strip()
    try:
        if action == "move":
            node.sport_move(
                act.get("vx", 0.0), act.get("vy", 0.0),
                act.get("wz", 0.0), act.get("duration", 1.5),
            )
        elif action == "stop":
            node.sport_stop()
        elif action == "stand":
            node.sport_stand()
        elif action == "gait":
            mode = str(act.get("mode", "")).lower()
            code = act.get("code", GAIT_CODES.get(mode))
            if code is None:
                return None, (f"unknown gait mode '{mode}'. "
                              f"Use one of: {', '.join(GAIT_CODES)}.")
            node.sport_gait(int(code))
            return reply or f"Switched to {mode or code} mode.", None
        elif action == "speed":
            level_name = str(act.get("level", "")).lower()
            level = act.get("value", SPEED_LEVELS.get(level_name))
            if level is None:
                return None, (f"unknown speed '{level_name}'. "
                              f"Use one of: {', '.join(SPEED_LEVELS)}.")
            node.sport_speed(int(level))
            return reply or f"Speed set to {level_name or level}.", None
        elif action == "damp":
            node.sport_damp()
        elif action == "lie_down":
            node.sport_lie_down()
        elif action == "recover":
            node.sport_recover()
        elif action == "teleop":
            running = node.teleop_running()
            want = bool(act.get("enable", True))
            if want != running:
                node.tap_f3()
            else:
                reply = reply or (
                    "Teleop is already running." if running else "Teleop is already off."
                )
        elif action == "list_topics":
            topics = node.topics()
            lines = "\n".join(f"• {n}  ({', '.join(t)})" for n, t in topics)
            return f"{len(topics)} ROS topics:\n{lines}", None
        elif action == "list_nodes":
            names = sorted(
                (f"{ns.rstrip('/')}/{n}" if ns not in ("", "/") else n)
                for n, ns in node.nodes()
            )
            lines = "\n".join(f"• {n}" for n in names)
            return f"{len(names)} ROS nodes:\n{lines}", None
        elif action == "battery":
            if node.battery_soc is None:
                return "Battery data not available yet (needs /lowstate).", None
            extra = []
            if node.battery_v is not None:
                extra.append(f"{node.battery_v:.1f} V")
            if node.battery_a is not None:
                extra.append(f"{node.battery_a:+.1f} A")
            tail = f" ({', '.join(extra)})" if extra else ""
            return f"Battery {node.battery_soc}%{tail}.", None
        elif action == "status":
            info = sys_info()
            soc = f"{node.battery_soc}%" if node.battery_soc is not None else "?"
            teleop = "on" if node.teleop_running() else "off"
            grip = node.gripper_state or "?"
            return (f"Status: battery {soc}, CPU temp {info['temp']}, "
                    f"RAM {info['ram']}, teleop {teleop}, gripper {grip}."), None
        elif action == "say":
            pass
        else:
            return None, f"unknown action: '{action}'"
    except Exception as exc:
        return None, f"failed to run '{action}': {exc}"
    return reply or f"OK ({action}).", None


def _agent_generate(model: str, message: str
                    ) -> Tuple[Optional[str], Optional[str]]:
    """One skill-driven generate call. Returns (raw_reply, error)."""
    payload = json.dumps({
        "model": model, "prompt": message,
        "system": load_skill(), "stream": False,
        "options": {"num_ctx": 2048, "num_predict": 200, "temperature": 0.1},
    }).encode("utf-8")
    req = urllib.request.Request(
        OLLAMA_URL + "/api/generate", data=payload,
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=180) as r:
            data = json.loads(r.read().decode("utf-8"))
        return (data.get("response", "") or "").strip(), None
    except urllib.error.HTTPError as exc:
        # 500 here is almost always the model failing to load — on this Orin that
        # means the CUDA/GPU allocation OOM'd (big models like 7B don't fit).
        hint = " (likely out of GPU memory — pick a smaller model)" if exc.code == 500 else ""
        return None, f"{model}: HTTP {exc.code}{hint}"
    except Exception as exc:
        return None, f"{model}: {exc}"


def is_vision_question(message: str) -> bool:
    """Recognize explicit questions that should inspect the current camera."""
    cleaned = " ".join(message.lower().strip().split())
    return any(re.search(pattern, cleaned) for pattern in VISION_PATTERNS)


def _vision_generate(model: str, message: str, jpeg: bytes
                     ) -> Tuple[Optional[str], Optional[str]]:
    """Ask the local VLM about one current forwarded-camera JPEG."""
    payload = json.dumps({
        "model": model,
        "messages": [
            {"role": "system", "content": VISION_SYSTEM},
            {
                "role": "user",
                "content": message,
                "images": [base64.b64encode(jpeg).decode("ascii")],
            },
        ],
        "stream": False,
        "keep_alive": "30m",
        "options": {
            "num_ctx": 4096,
            "num_predict": 160,
            "temperature": 0.1,
        },
    }).encode("utf-8")
    request = urllib.request.Request(
        OLLAMA_URL + "/api/chat",
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=180) as response:
            data = json.loads(response.read().decode("utf-8"))
        reply = " ".join(str(
            data.get("message", {}).get("content", "")).strip().split())
        if not reply:
            return None, "VLM returned an empty camera description."
        return reply, None
    except urllib.error.HTTPError as exc:
        hint = " (the selected model may not support images)" if exc.code == 400 else ""
        return None, f"{model}: VLM HTTP {exc.code}{hint}"
    except Exception as exc:
        return None, f"{model}: camera analysis failed: {exc}"


def pick_fallback_model(exclude: str) -> Optional[str]:
    """Smallest installed Ollama model that isn't the one that just failed —
    used to auto-recover when the chosen model OOMs. Size parsed from the name."""
    models = [m for m in list_ollama_models() if m != exclude]
    return sorted(models, key=model_param_billions)[0] if models else None


def route_agent(node: "MonitorNode", ollama_model: str, message: str
                ) -> Tuple[Optional[str], Optional[str]]:
    """Chat message → local LLM (with the robot skill) → JSON action → execute.
    If the chosen model errors (typically a 7B OOMing the Orin GPU), fall back to
    the smallest other installed model and tell the user."""
    if is_vision_question(message):
        frame = node.camera.snapshot()
        if frame is None:
            return None, (
                "Camera frame is unavailable. Wait until the camera card is "
                "visible in the web monitor and try again.")
        node.get_logger().info(
            f"VLM camera question using {len(frame)}-byte current JPEG: "
            f"{message}")
        return _vision_generate(ollama_model, message, frame)
    if node.pub_sport is None:
        return None, ("Motion control unavailable (unitree_api package is not "
                      "imported in this node).")
    raw, err = _agent_generate(ollama_model, message)
    note = ""
    if err:
        fb = pick_fallback_model(exclude=ollama_model)
        if not fb:
            return None, f"{err}. No fallback model installed."
        raw, err2 = _agent_generate(fb, message)
        if err2:
            return None, f"{err}; fallback {err2}."
        note = f"[{ollama_model} unavailable → used {fb}] "
    act = extract_action(raw)
    if act is None:
        return None, (f"LLM did not return a valid JSON action. "
                      f"Raw reply: {raw[:200]}")
    reply, aerr = execute_action(node, act)
    if aerr:
        return None, aerr
    return (note + (reply or "")), None


CSS = """
:root{color-scheme:dark}
body{font-family:system-ui,sans-serif;margin:0;background:#12141a;color:#e6e6e6}
header{position:sticky;top:0;z-index:900;background:#1c1f27;padding:14px 20px;
  display:flex;align-items:center;justify-content:space-between;gap:12px;
  flex-wrap:wrap;border-bottom:1px solid #2a2e39;box-shadow:0 5px 18px #0005}
h1{font-size:18px;margin:0}
.header-actions{display:flex;align-items:center;justify-content:flex-end;gap:8px;
  flex-wrap:wrap}
.header-clock{display:inline-flex;align-items:center;min-height:26px;padding:4px 9px;
  border:1px solid #343a48;border-radius:999px;background:#12141a;color:#f8fafc;
  font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:14px;
  font-weight:700;font-variant-numeric:tabular-nums;letter-spacing:.02em}
.header-ram,.header-battery{display:flex;align-items:center;padding:5px 10px 5px 9px;
  border:1px solid #343a48;border-radius:999px;background:#12141a;
  box-shadow:inset 0 1px 0 #ffffff0a}
.header-ram{cursor:help}
.header-ram .ram-gauge{gap:7px;min-height:20px}
.header-ram .ram-chip{width:30px;height:16px}
.header-ram .ram-percent{font-size:13px}
.header-battery .battery-gauge{gap:7px;min-height:20px}
.header-battery .battery-icon{width:34px;height:16px;border-radius:4px}
.header-battery .battery-icon::after{height:8px}
.header-battery .battery-percent{font-size:13px}
@media(max-width:640px){
  header{padding:10px 12px;gap:8px}
  h1{font-size:15px}
  .header-actions{gap:5px}
  .header-actions .btn.ghost{display:none}
  .header-actions button{padding:7px 10px;font-size:12px}
  .header-ram,.header-battery{padding:4px 9px 4px 8px}
}
main{padding:16px;max-width:1100px;margin:0 auto}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:16px}
@media(max-width:800px){.grid{grid-template-columns:1fr}}
.card{background:#1c1f27;border:1px solid #2a2e39;border-radius:10px;padding:14px;
  margin-bottom:16px}
.card h2{font-size:14px;margin:0 0 10px;color:#9aa4b2;text-transform:uppercase;
  letter-spacing:.05em}
table{width:100%;border-collapse:collapse;font-size:13px}
td{padding:4px 6px;border-bottom:1px solid #23262f;vertical-align:top}
td.k{color:#9aa4b2;width:42%}
.pill{display:inline-block;padding:2px 9px;border-radius:20px;font-size:12px;
  font-weight:600}
.ok{background:#123d2b;color:#4ade80}
.bad{background:#3d1620;color:#f87171}
.warn{background:#3d3416;color:#fbbf24}
.battery-gauge{display:inline-flex;align-items:center;gap:9px;min-height:26px;
  color:#6b7280}
.battery-icon{position:relative;display:inline-block;width:42px;height:20px;
  padding:2px;box-sizing:border-box;border:2px solid currentColor;border-radius:5px;
  background:#12141a}
.battery-icon::after{content:"";position:absolute;top:50%;right:-6px;width:4px;
  height:10px;transform:translateY(-50%);border-radius:0 2px 2px 0;
  background:currentColor}
.battery-fill{display:block;height:100%;border-radius:2px;background:currentColor;
  transition:width .35s ease}
.battery-percent{min-width:3.4ch;color:#e6e6e6;font-size:15px;font-weight:700;
  font-variant-numeric:tabular-nums}
.battery-good{color:#4ade80}
.battery-medium{color:#fbbf24}
.battery-low{color:#f87171}
.battery-unknown{color:#6b7280}
.ram-gauge{display:inline-flex;align-items:center;gap:9px;min-height:26px;
  color:#6b7280}
.ram-chip{position:relative;display:inline-block;width:38px;height:20px;padding:3px;
  box-sizing:border-box;border:2px solid currentColor;border-radius:4px;
  background:#12141a}
.ram-chip::before,.ram-chip::after{content:"";position:absolute;top:2px;width:3px;
  height:2px;background:currentColor;box-shadow:0 5px 0 currentColor,
  0 10px 0 currentColor}
.ram-chip::before{left:-5px}
.ram-chip::after{right:-5px}
.ram-fill{display:block;height:100%;border-radius:1px;background:currentColor;
  transition:width .35s ease}
.ram-reading{display:inline-flex;align-items:baseline;gap:4px;white-space:nowrap}
.ram-label{color:#a8b0c0;font-size:11px;font-weight:700;letter-spacing:.04em}
.ram-percent{min-width:3.4ch;color:#e6e6e6;font-size:15px;font-weight:700;
  font-variant-numeric:tabular-nums}
.ram-good{color:#4ade80}
.ram-medium{color:#fbbf24}
.ram-high{color:#f87171}
.ram-unknown{color:#6b7280}
.mono{font-family:ui-monospace,Menlo,monospace;font-size:12px}
.dup{background:#3d1620;color:#f87171;font-weight:700}
button,.btn{background:#2b64f5;color:#fff;border:0;padding:9px 16px;border-radius:8px;
  font-size:14px;cursor:pointer;text-decoration:none;display:inline-block}
button.stop{background:#c0392b}
button:disabled{opacity:.55;cursor:not-allowed}
.btn.ghost{background:#2a2e39}
img.cam{width:100%;border-radius:8px;background:#000;min-height:220px}
.audioctl{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.audioctl button.live{background:#c0392b}
.audiolevel{color:#8b93a2;font-size:12px}
.small{color:#6b7280;font-size:12px}
ul.nodes{list-style:none;padding:0;margin:0;font-size:13px}
ul.nodes li{padding:3px 6px;border-bottom:1px solid #23262f;display:flex;
  align-items:center;justify-content:space-between;gap:8px}
button.kill{background:#7a2531;color:#f3b0b7;border:0;padding:2px 10px;
  border-radius:6px;font-size:11px;cursor:pointer}
button.kill:hover{background:#c0392b;color:#fff}
.flash{background:#123d2b;color:#7ee2a8;border:1px solid #1c5c40;
  border-radius:8px;padding:10px 14px;margin-bottom:14px;font-size:13px}
.actionnotice{position:fixed;top:72px;right:18px;z-index:1000;max-width:min(440px,calc(100vw - 36px));
  padding:11px 14px;border-radius:9px;border:1px solid #3158a8;background:#17233d;
  color:#dbeafe;box-shadow:0 10px 30px #0008;font-size:13px;line-height:1.4;
  opacity:0;transform:translateY(-8px);pointer-events:none;transition:.18s ease}
.actionnotice.show{opacity:1;transform:translateY(0)}
.actionnotice.ok{background:#123d2b;border-color:#1c7550;color:#9af0ba}
.actionnotice.warn{background:#3d3416;border-color:#80691a;color:#fde68a}
.actionnotice.bad{background:#451c25;border-color:#943443;color:#fecdd3}
button[data-ajax-busy="1"]{opacity:.65;cursor:progress}
.inline{display:inline;margin:0}
.chatcard{border-color:#2b64f5;box-shadow:0 0 0 1px #2b64f533}
.chatcard h2{color:#7ea1ff}
.btnrow{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:6px}
.targetgrid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;
  margin:10px 0}
.targetfield{display:flex;flex-direction:column;gap:4px;min-width:0}
.targetfield label{font-size:11px;color:#a8b0c0}
.targetfield input,.targetmode{box-sizing:border-box;width:100%;background:#12141a;
  border:1px solid #2a2e39;color:#e6e6e6;padding:8px;border-radius:7px;
  font-family:ui-monospace,SFMono-Regular,Menlo,monospace}
@media(max-width:560px){.targetgrid{grid-template-columns:repeat(2,minmax(0,1fr))}}
.chatlog{min-height:180px;max-height:44vh;overflow-y:auto;display:flex;
  flex-direction:column;gap:8px;padding:6px 2px;margin-bottom:10px}
.msg.hint{background:transparent;color:#8b93a2;font-size:12px;max-width:100%;
  align-self:stretch;padding-left:0}
.msg{padding:8px 11px;border-radius:10px;font-size:13px;max-width:88%;white-space:pre-wrap;
  word-wrap:break-word;overflow-wrap:anywhere;line-height:1.45}
.msg.user{align-self:flex-end;background:#2b64f5;color:#fff;border-bottom-right-radius:3px}
.msg.ai{align-self:flex-start;background:#2a2e39;color:#e6e6e6;border-bottom-left-radius:3px}
.chatrow{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
.chatrow input{flex:1;min-width:150px;background:#12141a;border:1px solid #2a2e39;
  color:#e6e6e6;padding:9px 11px;border-radius:8px;font-size:14px}
.chatrow select{background:#12141a;border:1px solid #2a2e39;color:#e6e6e6;
  padding:9px;border-radius:8px;font-size:13px}
"""


# Client JS kept as a plain (non-f) string so its many { } braces need no
# escaping; interpolated into the page template as {SCRIPTS}.
SCRIPTS = """
<script>
let statusPollRunning=false;
const armTargetSchemas={
  JOINT:[['Joint 1','rad'],['Joint 2','rad'],['Joint 3','rad'],
         ['Joint 4','rad'],['Joint 5','rad'],['Joint 6','rad']],
  CARTESIAN:[['X world','m'],['Y world','m'],['Z world','m'],
             ['Roll world','rad'],['Pitch world','rad'],['Yaw world','rad']],
  CYLINDRICAL:[['Radius','m'],['Theta','rad'],['Z world','m'],
               ['Roll world','rad'],['Pitch world','rad'],['Yaw world','rad']]
};
let armTargetCurrent={};
let armTargetInputsDirty=false;
function armTargetMode(){
  return document.getElementById('arm_target_mode')?.value || 'JOINT';
}
function renderArmTargetCurrent(){
  const mode=armTargetMode();
  const key=mode.toLowerCase();
  const values=armTargetCurrent?.[key];
  const output=document.getElementById('arm_target_current_text');
  if(!output) return;
  output.textContent=Array.isArray(values) && values.length===6
    ? values.map((value)=>Number(value).toFixed(4)).join(', ')
    : 'feedback unavailable';
}
function fillArmTargetFromCurrent(){
  const values=armTargetCurrent?.[armTargetMode().toLowerCase()];
  if(!Array.isArray(values) || values.length!==6){
    showActionNotice('Current target feedback is unavailable.','warn',4500);
    return;
  }
  values.forEach((value,index)=>{
    const input=document.getElementById('arm_target_v'+(index+1));
    if(input) input.value=Number(value).toFixed(6);
  });
  armTargetInputsDirty=false;
  renderArmTargetCurrent();
}
function updateArmTargetSchema(fillCurrent=true){
  const schema=armTargetSchemas[armTargetMode()] || armTargetSchemas.JOINT;
  schema.forEach((item,index)=>{
    const label=document.getElementById('arm_target_label_'+(index+1));
    const input=document.getElementById('arm_target_v'+(index+1));
    if(label) label.textContent=item[0]+' ('+item[1]+')';
    if(input) input.placeholder=item[1];
  });
  armTargetInputsDirty=false;
  renderArmTargetCurrent();
  if(fillCurrent) fillArmTargetFromCurrent();
}
async function pollStatus(){
  if(statusPollRunning) return;
  statusPollRunning=true;
  try{
    const r = await fetch('/status.json',{cache:'no-store'});
    if(!r.ok) return;
    const d = await r.json();
    for(const k in d){ const el=document.getElementById('st_'+k); if(el) el.innerHTML=d[k]; }
    const restartButton=document.getElementById('restart_om6dof_btn');
    const armStackCard=document.getElementById('arm_stack_card');
    if(armStackCard && d.arm_stack!==undefined) armStackCard.innerHTML=d.arm_stack;
    if(restartButton && d.arm_stack_busy!==undefined){
      restartButton.disabled=!!d.arm_stack_busy;
      restartButton.textContent=d.arm_stack_busy ? '⏳ Restarting OM6DOF…' : '♻ Restart OM6DOF stack';
    }
    const perceptionCard=document.getElementById('perception_card');
    const startPerception=document.getElementById('start_perception_btn');
    const stopPerception=document.getElementById('stop_perception_btn');
    if(perceptionCard && d.perception!==undefined) perceptionCard.innerHTML=d.perception;
    if(d.perception_active!==undefined){
      if(startPerception) startPerception.disabled=!!d.perception_active;
      if(stopPerception) stopPerception.disabled=!d.perception_active;
    }
    const ddgngCard=document.getElementById('ddgng_card');
    const startDdgng=document.getElementById('start_ddgng_btn');
    const stopDdgng=document.getElementById('stop_ddgng_btn');
    if(ddgngCard && d.ddgng!==undefined) ddgngCard.innerHTML=d.ddgng;
    if(d.ddgng_active!==undefined){
      if(startDdgng) startDdgng.disabled=!!d.ddgng_active;
      if(stopDdgng) stopDdgng.disabled=!d.ddgng_active;
    }
    const pickupButton=document.getElementById('pickup_object_btn');
    if(pickupButton && d.pickup_busy!==undefined){
      pickupButton.disabled=!!d.pickup_busy || !!d.object_search_active || !d.perception_active;
      pickupButton.textContent=d.pickup_busy ? '⏳ Pickup running…' : '🤖 Pickup object';
    }
    const trackStart=document.getElementById('start_object_tracking_btn');
    const trackStop=document.getElementById('stop_object_tracking_btn');
    if(d.object_tracking_active!==undefined && d.object_tracking_busy!==undefined){
      if(trackStart) trackStart.disabled=!!d.object_tracking_active || !!d.object_tracking_busy || !!d.object_search_active || !d.perception_active;
      if(trackStop) trackStop.disabled=!d.object_tracking_active || !!d.object_tracking_busy;
    }
    const searchStart=document.getElementById('start_object_search_btn');
    const searchStop=document.getElementById('stop_object_search_btn');
    if(d.object_search_active!==undefined && d.object_search_busy!==undefined){
      if(searchStart) searchStart.disabled=!!d.object_search_active || !!d.object_search_busy || !!d.pickup_busy || !!d.object_tracking_active || !d.perception_active;
      if(searchStop) searchStop.disabled=!d.object_search_active || !!d.object_search_busy;
      if(searchStart) searchStart.textContent=(d.object_search_busy || d.object_search_active) ? '⏳ Searching object…' : '🔎 Start searching state';
    }
    if(d.arm_target_current && typeof d.arm_target_current==='object'){
      armTargetCurrent=d.arm_target_current;
      renderArmTargetCurrent();
      const inputs=[1,2,3,4,5,6].map((index)=>document.getElementById('arm_target_v'+index));
      const currentValues=armTargetCurrent?.[armTargetMode().toLowerCase()];
      if(Array.isArray(currentValues) && currentValues.length===6
          && !armTargetInputsDirty && inputs.some((input)=>input && input.value==='')){
        fillArmTargetFromCurrent();
      }
    }
    const armTargetSend=document.getElementById('send_arm_target_btn');
    const targetBlocked=!d.remote_enabled || !!d.arm_target_active || !!d.pickup_busy
      || !!d.object_tracking_active || !!d.object_tracking_busy
      || !!d.object_search_active || !!d.object_search_busy || !!d.arm_stack_busy
      || !['JOINT','CARTESIAN','CYLINDRICAL'].includes(d.control_mode_value);
    if(armTargetSend){
      armTargetSend.disabled=targetBlocked;
      armTargetSend.textContent=d.arm_target_active ? '⏳ Target running…' : '▶ Run target';
    }
    const cameraCard=document.getElementById('camera_card');
    const cameraImage=document.getElementById('camera_image');
    if(cameraCard && cameraImage && d.camera_available!==undefined){
      if(d.camera_available){
        cameraCard.style.display='block';
        if(!cameraImage.getAttribute('src')) cameraImage.src='/camera.mjpg?'+Date.now();
      }else{
        cameraCard.style.display='none';
        cameraImage.removeAttribute('src');
      }
    }
    const perceptionCameraCard=document.getElementById('perception_camera_card');
    const perceptionCameraImage=document.getElementById('perception_camera_image');
    const perceptionCameraStatus=document.getElementById('perception_camera_status');
    if(perceptionCameraCard && perceptionCameraImage && d.perception_camera_available!==undefined){
      perceptionCameraCard.style.display='block';
      if(d.perception_camera_available){
        perceptionCameraImage.style.display='block';
        if(!perceptionCameraImage.getAttribute('src')) perceptionCameraImage.src='/perception.mjpg?'+Date.now();
        if(perceptionCameraStatus) perceptionCameraStatus.textContent='RealSense processing stream active.';
      }else{
        perceptionCameraImage.style.display='none';
        perceptionCameraImage.removeAttribute('src');
        if(perceptionCameraStatus) perceptionCameraStatus.textContent='Waiting for RealSense perception. Press Start perception.';
      }
    }
    const ddgngCameraCard=document.getElementById('ddgng_camera_card');
    const ddgngCameraImage=document.getElementById('ddgng_camera_image');
    const ddgngCameraStatus=document.getElementById('ddgng_camera_status');
    if(ddgngCameraCard && ddgngCameraImage && d.ddgng_camera_available!==undefined){
      ddgngCameraCard.style.display='block';
      if(d.ddgng_camera_available){
        ddgngCameraImage.style.display='block';
        if(!ddgngCameraImage.getAttribute('src')) ddgngCameraImage.src='/ddgng.mjpg?'+Date.now();
        if(ddgngCameraStatus) ddgngCameraStatus.textContent='DD-GNG YOLO semantic stream active.';
      }else{
        ddgngCameraImage.style.display='none';
        ddgngCameraImage.removeAttribute('src');
        if(ddgngCameraStatus) ddgngCameraStatus.textContent='Waiting for DD-GNG YOLO. Press Start DD-GNG YOLO.';
      }
    }
    const audioCard=document.getElementById('audio_card');
    if(audioCard && d.audio_available!==undefined){
      audioCard.style.display=d.audio_available ? 'block' : 'none';
      if(!d.audio_available && liveAudioController) stopLiveAudio();
    }
    const voiceBadge=document.getElementById('voice_badge');
    if(voiceBadge && d.voice_active!==undefined){
      voiceBadge.textContent=d.voice_active ? 'VOICE DETECTED' : 'NO VOICE DETECTED';
      voiceBadge.className='pill '+(d.voice_active ? 'ok' : 'warn');
    }
    const dot=document.getElementById('st_dot');
    if(dot){ dot.style.color='#4ade80'; setTimeout(()=>{dot.style.color='#6b7280';},400); }
    document.querySelectorAll('button[data-ajax-busy="1"]').forEach((b)=>{b.disabled=true;});
  }catch(e){}
  finally{ statusPollRunning=false; }
}
setInterval(pollStatus,1000); pollStatus();

function updateHeaderClock(){
  const clock=document.getElementById('header_clock');
  if(!clock) return;
  const now=new Date();
  const pad=(value)=>String(value).padStart(2,'0');
  clock.textContent=pad(now.getHours())+':'+pad(now.getMinutes());
  clock.dateTime=now.toISOString();
  try{
    clock.title=new Intl.DateTimeFormat('en-US',{
      dateStyle:'full',timeStyle:'medium'
    }).format(now);
  }catch(_error){ clock.title=now.toLocaleString(); }
}
setInterval(updateHeaderClock,1000); updateHeaderClock();

let actionNoticeTimer=null;
function showActionNotice(message,kind='info',holdMs=4500){
  const notice=document.getElementById('action_notice');
  if(!notice) return;
  if(actionNoticeTimer) clearTimeout(actionNoticeTimer);
  notice.textContent=message;
  notice.className='actionnotice show '+kind;
  notice.setAttribute('role',kind==='bad' ? 'alert' : 'status');
  if(holdMs>0){
    actionNoticeTimer=setTimeout(()=>{notice.classList.remove('show');},holdMs);
  }
}

function responseMessageFromHtml(text,status){
  try{
    const doc=new DOMParser().parseFromString(text,'text/html');
    const message=[doc.querySelector('h2')?.textContent,doc.querySelector('p')?.textContent]
      .filter(Boolean).join(' — ').trim();
    if(message) return message;
  }catch(_error){}
  return 'HTTP '+status;
}

function refreshStatusBurst(){
  [0,250,750,1500,3000].forEach((delay)=>setTimeout(pollStatus,delay));
}

async function submitControlForm(form,submitter){
  if(form.dataset.ajaxBusy==='1') return;
  const button=submitter || form.querySelector('button[type="submit"],input[type="submit"]');
  const originalText=button ? button.textContent : '';
  form.dataset.ajaxBusy='1';
  form.setAttribute('aria-busy','true');
  if(button){
    button.dataset.ajaxBusy='1';
    button.disabled=true;
    button.textContent='⏳ Processing…';
  }
  showActionNotice('Sending command…','info',0);
  const body=new URLSearchParams();
  for(const [key,value] of new FormData(form).entries()) body.append(key,String(value));
  const action=new URL(form.action,window.location.href);
  try{
    const response=await fetch(action.pathname+action.search,{
      method:'POST',
      credentials:'same-origin',
      cache:'no-store',
      redirect:'follow',
      headers:{
        'Accept':'application/json',
        'Content-Type':'application/x-www-form-urlencoded;charset=UTF-8',
        'X-Requested-With':'fetch'
      },
      body:body.toString()
    });
    const contentType=response.headers.get('content-type') || '';
    let message='Command accepted.';
    let actionOk=response.ok;
    if(contentType.includes('application/json')){
      const payload=await response.json();
      message=payload.message || payload.error || message;
      if(payload.ok!==undefined) actionOk=actionOk && !!payload.ok;
    }else{
      message=responseMessageFromHtml(await response.text(),response.status);
    }
    showActionNotice(message,actionOk ? 'ok' : 'bad',actionOk ? 4500 : 8000);
  }catch(error){
    const disruptive=action.pathname==='/restart' || action.pathname==='/shutdown';
    showActionNotice(
      disruptive ? 'Monitor is restarting or shutting down; waiting for the connection…' : ('Request failed: '+error),
      disruptive ? 'warn' : 'bad',disruptive ? 8000 : 10000);
  }finally{
    form.dataset.ajaxBusy='0';
    form.removeAttribute('aria-busy');
    if(button){
      delete button.dataset.ajaxBusy;
      button.textContent=originalText;
    }
    refreshStatusBurst();
  }
}

let liveAudioController=null;
let liveAudioContext=null;
let liveAudioNextTime=0;

function setAudioUi(enabled,label){
  const button=document.getElementById('audio_toggle');
  const status=document.getElementById('audio_status');
  if(button){
    button.textContent=enabled ? '🔇 Disable live audio' : '🔊 Enable live audio';
    button.classList.toggle('live',enabled);
  }
  if(status) status.textContent=label || (enabled ? 'playing microphone' : 'muted');
}

async function toggleLiveAudio(){
  if(liveAudioController){ stopLiveAudio(); return; }
  const AudioCtx=window.AudioContext||window.webkitAudioContext;
  if(!AudioCtx){ setAudioUi(false,'Web Audio is not supported'); return; }
  const controller=new AbortController();
  const context=new AudioCtx();
  liveAudioController=controller;
  liveAudioContext=context;
  liveAudioNextTime=context.currentTime+0.08;
  setAudioUi(true,'connecting…');
  try{
    await context.resume();
    const response=await fetch('/audio.pcm?'+Date.now(),{
      cache:'no-store',signal:controller.signal
    });
    if(!response.ok || !response.body) throw new Error('audio HTTP '+response.status);
    const reader=response.body.getReader();
    let remainder=new Uint8Array(0);
    setAudioUi(true,'live · mono 48 kHz');
    while(true){
      const result=await reader.read();
      if(result.done) break;
      let data=result.value;
      if(remainder.length){
        const joined=new Uint8Array(remainder.length+data.length);
        joined.set(remainder); joined.set(data,remainder.length); data=joined;
      }
      const byteCount=data.length-(data.length%2);
      remainder=data.slice(byteCount);
      if(!byteCount) continue;
      const samples=byteCount/2;
      const audioBuffer=context.createBuffer(1,samples,48000);
      const channel=audioBuffer.getChannelData(0);
      const view=new DataView(data.buffer,data.byteOffset,byteCount);
      for(let i=0;i<samples;i++) channel[i]=view.getInt16(i*2,true)/32768;
      const source=context.createBufferSource();
      source.buffer=audioBuffer; source.connect(context.destination);
      if(liveAudioNextTime<context.currentTime+0.03 ||
         liveAudioNextTime>context.currentTime+0.5){
        liveAudioNextTime=context.currentTime+0.06;
      }
      source.start(liveAudioNextTime);
      liveAudioNextTime+=audioBuffer.duration;
    }
  }catch(error){
    if(error.name!=='AbortError') setAudioUi(false,'audio error: '+error.message);
  }finally{
    if(liveAudioController===controller) stopLiveAudio();
  }
}

function stopLiveAudio(){
  const controller=liveAudioController;
  const context=liveAudioContext;
  liveAudioController=null; liveAudioContext=null; liveAudioNextTime=0;
  if(controller) controller.abort();
  if(context) context.close().catch(()=>{});
  setAudioUi(false,'muted · STT remains active');
}

function _log(){ return document.getElementById('chatlog'); }
function addMsg(who,text){
  const d=document.createElement('div'); d.className='msg '+who; d.textContent=text;
  _log().appendChild(d); _log().scrollTop=_log().scrollHeight; return d;
}
async function sendChat(){
  const inp=document.getElementById('chatinput');
  const msg=inp.value.trim(); if(!msg) return;
  const model=document.getElementById('chatmodel').value;
  addMsg('user',msg); inp.value='';
  const t=addMsg('ai','…'); t.style.opacity='0.6';
  try{
    const r=await fetch('/chat',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({model:model,message:msg})});
    const d=await r.json();
    t.style.opacity='1'; t.textContent = d.error ? ('⚠ '+d.error) : d.reply;
  }catch(e){ t.style.opacity='1'; t.textContent='⚠ '+e; }
  _log().scrollTop=_log().scrollHeight;
}
// Preload (warm up) the selected Ollama model so the first chat is fast. Fired
// when the user first focuses the input or switches model. Keyed per model so we
// only load each once.
const _preloaded={};
async function preloadModel(){
  const sel=document.getElementById('chatmodel'); if(!sel) return;
  const model=sel.value;
  if(!model || _preloaded[model]) return;
  if(!(model.startsWith('agent:')||model.startsWith('ollama:'))) return; // skip codex/claude
  _preloaded[model]=true;
  const s=document.getElementById('chatstatus');
  if(s) s.textContent='⏳ loading model into memory…';
  try{
    const r=await fetch('/preload_llm',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({model:model})});
    const d=await r.json();
    if(s) s.textContent = d.ok ? '✓ model ready' : ('⚠ '+(d.model||'load failed'));
    if(!d.ok) _preloaded[model]=false;  // allow a retry
  }catch(e){ _preloaded[model]=false; if(s) s.textContent=''; }
  if(s) setTimeout(()=>{ if(s.textContent==='✓ model ready') s.textContent=''; },2500);
}
document.addEventListener('DOMContentLoaded',function(){
  document.addEventListener('submit',function(event){
    const form=event.target;
    if(!(form instanceof HTMLFormElement) || form.method.toLowerCase()!=='post') return;
    // Inline safety confirmations run on the target before this bubble
    // listener. A cancelled confirmation therefore remains cancelled.
    if(event.defaultPrevented) return;
    event.preventDefault();
    submitControlForm(form,event.submitter);
  });
  const b=document.getElementById('chatsend'); if(b) b.addEventListener('click',sendChat);
  const i=document.getElementById('chatinput');
  if(i){
    i.addEventListener('keydown',function(e){ if(e.key==='Enter') sendChat(); });
    i.addEventListener('focus',preloadModel,{once:true});  // warm up on first chat
  }
  const m=document.getElementById('chatmodel');
  if(m) m.addEventListener('change',preloadModel);         // and when model switches
  const targetMode=document.getElementById('arm_target_mode');
  if(targetMode) targetMode.addEventListener('change',()=>updateArmTargetSchema(true));
  for(let index=1;index<=6;index++){
    const input=document.getElementById('arm_target_v'+index);
    if(input) input.addEventListener('input',()=>{armTargetInputsDirty=true;});
  }
  const useCurrent=document.getElementById('arm_target_use_current');
  if(useCurrent) useCurrent.addEventListener('click',fillArmTargetFromCurrent);
  updateArmTargetSchema(false);
});
</script>
"""


def pill(ok: bool, yes: str, no: str) -> str:
    cls = "ok" if ok else "bad"
    return f'<span class="pill {cls}">{yes if ok else no}</span>'


def gripper_pill(state: Optional[str]) -> str:
    """Colour the gripper state string (e.g. 'HOLDING pos=-0.31 cur=95mA')."""
    if not state:
        return '<span class="pill warn">unknown</span>'
    word = state.split()[0].upper()
    cls = {
        "HOLDING": "ok", "OPEN": "ok",
        "DROPPED": "bad",
        "CLOSED": "warn", "CLOSING": "warn", "OPENING": "warn", "MID": "warn",
    }.get(word, "warn")
    return f'<span class="pill {cls}">{html.escape(state)}</span>'


def battery_pill(node) -> str:
    """Render a compact live battery gauge; electrical details stay a tooltip."""
    soc = node.battery_soc
    if soc is None:
        return (
            '<span class="battery-gauge battery-unknown" role="img" '
            'aria-label="Battery data unavailable" title="Battery data unavailable">'
            '<span class="battery-icon" aria-hidden="true">'
            '<span class="battery-fill" style="width:0%"></span></span>'
            '<strong class="battery-percent">--%</strong></span>'
        )
    level = max(0, min(100, int(soc)))
    cls = (
        "battery-good" if level >= 40
        else ("battery-medium" if level >= 20 else "battery-low")
    )
    details = [f"Battery {level}%"]
    if node.battery_v is not None:
        details.append(f"{node.battery_v:.1f} V")
    if node.battery_a is not None:
        details.append(f"{node.battery_a:+.1f} A")
    label = html.escape(" · ".join(details), quote=True)
    return (
        f'<span class="battery-gauge {cls}" role="img" '
        f'aria-label="{label}" title="{label}">'
        '<span class="battery-icon" aria-hidden="true">'
        f'<span class="battery-fill" style="width:{level}%"></span></span>'
        f'<strong class="battery-percent">{level}%</strong></span>'
    )


def ram_gauge(info: dict) -> str:
    """Render RAM usage as a compact memory-chip gauge for the top ribbon."""
    raw_percent = info.get("ram_percent")
    if raw_percent is None:
        return (
            '<span class="ram-gauge ram-unknown" role="img" '
            'aria-label="RAM data unavailable" '
            'title="RAM data unavailable">'
            '<span class="ram-chip" aria-hidden="true">'
            '<span class="ram-fill" style="width:0%"></span></span>'
            '<span class="ram-reading"><span class="ram-label">RAM</span> '
            '<strong class="ram-percent">--%</strong></span></span>'
        )
    level = max(0, min(100, int(raw_percent)))
    cls = (
        "ram-good" if level < 70
        else ("ram-medium" if level < 85 else "ram-high")
    )
    details = str(info.get("ram") or f"{level}% used")
    label = html.escape(
        f"RAM usage {level}% · {details}", quote=True
    )
    return (
        f'<span class="ram-gauge {cls}" role="img" '
        f'aria-label="{label}" title="{label}">'
        '<span class="ram-chip" aria-hidden="true">'
        f'<span class="ram-fill" style="width:{level}%"></span></span>'
        '<span class="ram-reading"><span class="ram-label">RAM</span> '
        f'<strong class="ram-percent">{level}%</strong></span></span>'
    )


def status_fields(node, cam=None, audio=None) -> dict:
    """The live, cheap-to-compute status cells (HTML strings). Used both for the
    initial page render and the /status.json poll. One graph query (node names)
    plus /proc reads and cached battery/gripper — light enough to poll often."""
    info = sys_info()
    counts: dict = {}
    node_names = set()
    for name, ns in node.nodes():
        node_names.add(name)
        key = f"{ns.rstrip('/')}/{name}" if ns not in ("", "/") else name
        counts[key] = counts.get(key, 0) + 1
    dups = sum(1 for c in counts.values() if c > 1)
    teleop_on = node.remote_enabled is True
    arm_target_state = str(
        getattr(node, "arm_target_state", "idle")
    ).strip().lower()
    arm_target_active = bool(getattr(node, "arm_target_active", False))
    arm_target_class = (
        "warn" if arm_target_active or arm_target_state in ("requesting", "stopping")
        else "ok" if arm_target_state in ("idle", "reached", "stopped")
        else "bad" if arm_target_state in ("rejected", "timeout", "blocked")
        else "warn"
    )
    running_llm = list_running_ollama()
    if running_llm:
        llm_cell = (f'<span class="pill warn">{html.escape(", ".join(running_llm))}</span>'
                    ' <form class="inline" method="POST" action="/stop_llm">'
                    '<button class="kill" type="submit">kill</button></form>')
    else:
        llm_cell = '<span class="pill ok">none loaded</span>'
    ssh_host = getattr(node, "robot_ssh_host", "")
    service = (
        om6dof_service_status(ssh_host)
        if ssh_host else om6dof_service_status()
    )
    perception_service = (
        perception_service_status(ssh_host)
        if ssh_host else perception_service_status()
    )
    ddgng_service = (
        ddgng_service_status(ssh_host)
        if ssh_host else ddgng_service_status()
    )
    missing = sorted(OM6DOF_REQUIRED_NODES - node_names)
    controller_issues = node.arm_controller_issues()
    restart = node.arm_restart_snapshot()
    restart_busy = restart["phase"] == "restarting"
    if restart_busy:
        arm_stack = '<span class="pill warn">RESTARTING</span>'
    elif (
        service["active_state"] == "active"
        and not missing
        and not controller_issues
    ):
        arm_stack = (
            '<span class="pill ok">ACTIVE / HEALTHY</span> '
            f'<span class="mono">PID {service["main_pid"]}</span>'
        )
    elif service["active_state"] == "active":
        issues = [f"missing node: {name}" for name in missing]
        issues.extend(controller_issues)
        arm_stack = (
            '<span class="pill bad">ACTIVE / INCOMPLETE</span> '
            f'<span class="mono">{html.escape("; ".join(issues))}</span>'
        )
    else:
        state = html.escape(
            f"{service['active_state']}/{service['sub_state']}"
        )
        arm_stack = f'<span class="pill bad">{state}</span>'
    perception_node_present = "om6dof_perception" in node_names
    if (
        perception_service["active_state"] == "active"
        and perception_node_present
    ):
        perception = (
            '<span class="pill ok">ACTIVE</span> '
            f'<span class="mono">PID {perception_service["main_pid"]}</span>'
        )
    elif perception_service["active_state"] == "active":
        perception = '<span class="pill warn">STARTING / NODE MISSING</span>'
    elif perception_service["active_state"] == "inactive":
        perception = '<span class="pill warn">STOPPED</span>'
    else:
        state = html.escape(
            f"{perception_service['active_state']}/"
            f"{perception_service['sub_state']}"
        )
        perception = f'<span class="pill bad">{state}</span>'
    if ddgng_service["active_state"] == "active":
        ddgng = (
            '<span class="pill ok">ACTIVE</span> '
            f'<span class="mono">PID {ddgng_service["main_pid"]}</span>'
        )
    elif ddgng_service["active_state"] == "inactive":
        ddgng = '<span class="pill warn">STOPPED</span>'
    else:
        state = html.escape(
            f"{ddgng_service['active_state']}/{ddgng_service['sub_state']}"
        )
        ddgng = f'<span class="pill bad">{state}</span>'
    camera_available = bool(cam is not None and cam.available())
    perception_cam = getattr(node, "perception_camera", None)
    perception_camera_available = bool(
        perception_cam is not None and perception_cam.available()
    )
    ddgng_cam = getattr(node, "ddgng_camera", None)
    ddgng_camera_available = bool(
        ddgng_cam is not None and ddgng_cam.available()
    )
    audio_available = bool(audio is not None and audio.available())
    return {
        "uptime": html.escape(info["uptime"]),
        "load": f'<span class="mono">{html.escape(info["load"])}</span>',
        "ram": ram_gauge(info),
        "temp": html.escape(info["temp"]),
        "battery": battery_pill(node),
        "arm_bus": pill(info["arm_bus"], "present", "MISSING"),
        "teleop": pill(teleop_on, "REMOTE ENABLED", "remote disabled"),
        "remote_enabled": teleop_on,
        "control_mode": (
            f'<span class="pill ok">{html.escape(node.control_mode)}</span>'
            if node.control_mode else '<span class="pill warn">unknown</span>'
        ),
        "control_mode_value": node.control_mode or "",
        "gripper": gripper_pill(node.gripper_state),
        "arm_stack": arm_stack,
        "perception": perception,
        "perception_active": (
            perception_service["active_state"] == "active"
        ),
        "ddgng": ddgng,
        "ddgng_active": ddgng_service["active_state"] == "active",
        "perception_tracking": html.escape(
            getattr(node, "perception_tracking_status", "offline")
        ),
        "perception_distance": (
            f'<span class="pill ok">{node.perception_distance_m:.3f} m</span>'
            if getattr(node, "perception_distance_m", None) is not None
            else '<span class="pill warn">unavailable</span>'
        ),
        "pickup_busy": bool(getattr(node, "pickup_busy", False)),
        "pickup_message": html.escape(
            getattr(node, "pickup_message", "not run yet")
        ),
        "object_tracking_active": bool(getattr(
            node, "object_tracking_active", False)),
        "object_tracking_busy": bool(getattr(
            node, "object_tracking_busy", False)),
        "object_tracking_message": html.escape(getattr(
            node, "object_tracking_message", "not run yet")),
        "object_search_active": bool(getattr(
            node, "object_search_active", False)),
        "object_search_busy": bool(getattr(
            node, "object_search_busy", False)),
        "object_search_message": html.escape(getattr(
            node, "object_search_message", "not run yet")),
        "arm_target_active": arm_target_active,
        "arm_target_state": (
            f'<span class="pill {arm_target_class}">'
            f'{html.escape(arm_target_state.upper())}</span>'
        ),
        "arm_target_mode": html.escape(
            getattr(node, "arm_target_mode", "") or "—"
        ),
        "arm_target_message": html.escape(
            getattr(node, "arm_target_message", "no target yet")
        ),
        "arm_target_current": getattr(node, "arm_target_current", {}),
        "arm_stack_busy": restart_busy,
        "arm_restart_message": (
            html.escape(restart["message"])
            if restart["message"] else "none yet"
        ),
        "llm": llm_cell,
        "dups": (f'<span class="pill bad">{dups} FOUND</span>' if dups
                 else '<span class="pill ok">none</span>'),
        "camera_available": camera_available,
        "perception_camera_available": perception_camera_available,
        "ddgng_camera_available": ddgng_camera_available,
        "audio_available": audio_available,
        "audio_source": html.escape(
            getattr(node, "audio_source", "unknown")),
        "voice_active": bool(getattr(node, "voice_active", False)),
        "stt_text": html.escape(
            getattr(node, "stt_text", "") or "No transcript yet."),
        "stt_status": html.escape(getattr(node, "stt_status", "offline")),
        "voice_llm_response": html.escape(
            getattr(node, "voice_llm_response", "")
            or "No LLM response yet."),
        "voice_llm_status": html.escape(
            getattr(node, "voice_llm_status", "offline")),
        "tts_status": html.escape(getattr(node, "tts_status", "offline")),
        "tts_state": (
            "speaking · microphone paused"
            if getattr(node, "tts_speaking", False)
            else "listening"),
    }


def render_page(
        node: MonitorNode,
        cam: ForwardedImageStream,
        audio: Optional[ForwardedPcmStream] = None) -> str:
    info = sys_info()
    nodes = node.nodes()

    # duplicate detection
    counts: dict = {}
    for name, ns in nodes:
        key = f"{ns.rstrip('/')}/{name}" if ns not in ("", "/") else name
        counts[key] = counts.get(key, 0) + 1
    dups = {k: c for k, c in counts.items() if c > 1}
    teleop_on = node.remote_enabled is True

    # --- status card (dynamic cells carry id="st_*" and are live-updated by
    # the poller script below hitting /status.json; static cells are plain) ---
    fields = status_fields(node, cam, audio)
    rows = [
        ("Hostname", html.escape(info["hostname"]), None),
        ("IP addresses", f'<span class="mono">{html.escape(info["ips"])}</span>', None),
        ("Uptime", fields["uptime"], "uptime"),
        ("Load avg", fields["load"], "load"),
        ("CPU temp", fields["temp"], "temp"),
        ("ROS_DOMAIN_ID", html.escape(str(info["domain_id"])), None),
        (f"Arm bus {ARM_BUS_DEVICE}", fields["arm_bus"], "arm_bus"),
        ("Teleop", fields["teleop"], "teleop"),
        ("Arm control mode", fields["control_mode"], "control_mode"),
        ("Gripper", fields["gripper"], "gripper"),
        ("OM6DOF stack", fields["arm_stack"], "arm_stack"),
        ("OM6DOF perception", fields["perception"], "perception"),
        ("OM6DOF DD-GNG YOLO", fields["ddgng"], "ddgng"),
        ("LLM loaded", fields["llm"], "llm"),
        ("Duplicate nodes", fields["dups"], "dups"),
    ]
    status_rows = "".join(
        f'<tr><td class="k">{k}</td>'
        + (f'<td id="st_{key}">{v}</td>' if key else f'<td>{v}</td>')
        + '</tr>'
        for k, v, key in rows
    )

    # --- duplicate warning banner ---
    dup_banner = ""
    if dups:
        items = "".join(
            f"<li class='dup'>{html.escape(k)} ×{c}</li>" for k, c in dups.items()
        )
        dup_banner = (
            '<div class="card"><h2>⚠ Duplicate nodes detected</h2>'
            f'<ul class="nodes">{items}</ul>'
            '<p class="small">Two nodes sharing a name usually means a stray '
            'process — kill the extra one.</p></div>'
        )

    # Node/topic lists are no longer rendered — ask the Robot Agent chat instead
    # ("list ROS topics" / "list ROS nodes"). We still compute `dups`/`counts`
    # above for the duplicate-node safety banner.

    # --- forwarded camera + teleop control ---
    camera_available = cam.available()
    cam_display = "block" if camera_available else "none"
    cam_src = ' src="/camera.mjpg"' if camera_available else ""
    cam_html = f"""
    <div class="card" id="camera_card" style="display:{cam_display}">
      <h2>📷 Go2W built-in camera</h2>
      <img class="cam" id="camera_image"{cam_src} alt="forwarded camera stream">
      <p class="small">ROS input: <span class="mono">{html.escape(cam.topic)}</span>.
      This monitor does not open the camera device.</p>
    </div>
    """
    perception_cam = getattr(node, "perception_camera", None)
    perception_camera_available = bool(
        perception_cam is not None and perception_cam.available()
    )
    perception_cam_src = (
        ' src="/perception.mjpg"' if perception_camera_available else ""
    )
    perception_img_display = (
        "block" if perception_camera_available else "none"
    )
    perception_cam_topic = html.escape(
        perception_cam.topic if perception_cam is not None else ""
    )
    perception_cam_html = f"""
    <div class="card" id="perception_camera_card">
      <h2>🎯 RealSense perception preview</h2>
      <img class="cam" id="perception_camera_image"{perception_cam_src}
           style="display:{perception_img_display}"
           alt="RealSense perception stream">
      <p id="perception_camera_status">{
          "RealSense processing stream active."
          if perception_camera_available
          else "Waiting for RealSense perception. Press Start perception."
      }</p>
      <p class="small">ROS input:
      <span class="mono">{perception_cam_topic}</span>. This stream is separate
      from the Go2W built-in camera.</p>
    </div>
    """
    ddgng_cam = getattr(node, "ddgng_camera", None)
    ddgng_camera_available = bool(
        ddgng_cam is not None and ddgng_cam.available()
    )
    ddgng_cam_src = ' src="/ddgng.mjpg"' if ddgng_camera_available else ""
    ddgng_img_display = "block" if ddgng_camera_available else "none"
    ddgng_cam_topic = html.escape(
        ddgng_cam.topic if ddgng_cam is not None else ""
    )
    ddgng_cam_html = f"""
    <div class="card" id="ddgng_camera_card">
      <h2>🕸️ OM6DOF DD-GNG YOLO preview</h2>
      <img class="cam" id="ddgng_camera_image"{ddgng_cam_src}
           style="display:{ddgng_img_display}"
           alt="OM6DOF DD-GNG RealSense stream">
      <p id="ddgng_camera_status">{
          "DD-GNG YOLO semantic stream active."
          if ddgng_camera_available
          else "Waiting for DD-GNG YOLO. Press Start DD-GNG YOLO."
      }</p>
      <p class="small">ROS input:
      <span class="mono">{ddgng_cam_topic}</span>. DD-GNG and perception take
      turns using the RealSense camera.</p>
    </div>
    """
    audio_available = bool(audio is not None and audio.available())
    audio_display = "block" if audio_available else "none"
    audio_topic = html.escape(audio.topic if audio is not None else "")
    voice_active = bool(getattr(node, "voice_active", False))
    voice_class = "ok" if voice_active else "warn"
    voice_text = "VOICE DETECTED" if voice_active else "NO VOICE DETECTED"
    audio_html = f"""
    <div class="card" id="audio_card" style="display:{audio_display}">
      <h2>🎙️ Live microphone</h2>
      <p>Source: <span class="mono" id="st_audio_source">{html.escape(getattr(node, "audio_source", "unknown"))}</span></p>
      <p><span class="pill {voice_class}" id="voice_badge">{voice_text}</span></p>
      <div class="audioctl">
        <button id="audio_toggle" type="button" onclick="toggleLiveAudio()">
          🔊 Enable live audio
        </button>
        <span class="audiolevel" id="audio_status">muted · STT remains active</span>
      </div>
      <p><strong>Speech to text</strong> ·
        <span class="pill warn" id="st_stt_status">{html.escape(getattr(node, "stt_status", "offline"))}</span>
      </p>
      <p class="mono" id="st_stt_text">{html.escape(getattr(node, "stt_text", "") or "No transcript yet.")}</p>
      <p><strong>Voice Robot Agent response</strong> ·
        <span class="pill warn" id="st_voice_llm_status">{html.escape(getattr(node, "voice_llm_status", "offline"))}</span>
      </p>
      <p class="mono" id="st_voice_llm_response">{html.escape(getattr(node, "voice_llm_response", "") or "No LLM response yet.")}</p>
      <p><strong>Natural robot voice</strong> ·
        <span class="pill warn" id="st_tts_status">{html.escape(getattr(node, "tts_status", "offline"))}</span>
        · <span id="st_tts_state">{"speaking · microphone paused" if getattr(node, "tts_speaking", False) else "listening"}</span>
      </p>
      <p class="small">PCM input: <span class="mono">{audio_topic}</span>.
      This button controls playback only in this browser; speech processing on
      the AGX remains active.</p>
    </div>
    """
    teleop_html = f"""
    <div class="card">
      <h2>🎮 Remote control</h2>
      <p>Status: {pill(teleop_on, "REMOTE ENABLED", "remote disabled")}</p>
      <div class="btnrow">
        <form class="inline" method="POST" action="/start_teleop">
          <button type="submit">▶ Enable remote</button>
        </form>
        <form class="inline" method="POST" action="/stop_teleop">
          <button class="stop" type="submit">■ Disable remote</button>
        </form>
      </div>
      <div class="btnrow">
        <form class="inline" method="POST" action="/mode_joint">
          <button type="submit">JOINT</button>
        </form>
        <form class="inline" method="POST" action="/mode_cartesian">
          <button type="submit">CARTESIAN</button>
        </form>
        <form class="inline" method="POST" action="/mode_cylindrical">
          <button type="submit">CYLINDRICAL</button>
        </form>
      </div>
      <p class="small">Sends a momentary F3 to /wirelesscontroller. Remote ON
      activates the forward position controller and ramps the arm to READY in
      JOINT mode; Remote OFF restores the trajectory controller for autonomous
      MoveIt execution. Select on the remote cycles JOINT → CARTESIAN →
      CYLINDRICAL.</p>
    </div>
    """

    initial_target_mode = (
        node.control_mode
        if node.control_mode in ("JOINT", "CARTESIAN", "CYLINDRICAL")
        else "JOINT"
    )
    initial_target_values = getattr(node, "arm_target_current", {}).get(
        initial_target_mode.lower(), []
    )
    initial_value_attrs = [
        f' value="{float(value):.6f}"'
        for value in initial_target_values
    ] if len(initial_target_values) == 6 else [""] * 6
    initial_labels = {
        "JOINT": [
            ("Joint 1", "rad"), ("Joint 2", "rad"),
            ("Joint 3", "rad"), ("Joint 4", "rad"),
            ("Joint 5", "rad"), ("Joint 6", "rad"),
        ],
        "CARTESIAN": [
            ("X world", "m"), ("Y world", "m"), ("Z world", "m"),
            ("Roll world", "rad"), ("Pitch world", "rad"),
            ("Yaw world", "rad"),
        ],
        "CYLINDRICAL": [
            ("Radius", "m"), ("Theta", "rad"), ("Z world", "m"),
            ("Roll world", "rad"), ("Pitch world", "rad"),
            ("Yaw world", "rad"),
        ],
    }[initial_target_mode]
    target_inputs = "".join(
        '<div class="targetfield">'
        f'<label id="arm_target_label_{index}">{html.escape(label)} '
        f'({html.escape(unit)})</label>'
        f'<input id="arm_target_v{index}" name="value_{index}" '
        f'type="number" step="any" inputmode="decimal" required'
        f'{initial_value_attrs[index - 1]} placeholder="{html.escape(unit)}">'
        '</div>'
        for index, (label, unit) in enumerate(initial_labels, start=1)
    )
    current_target_text = (
        ", ".join(f"{float(value):.4f}" for value in initial_target_values)
        if len(initial_target_values) == 6 else "feedback unavailable"
    )
    target_send_blocked = (
        not teleop_on
        or fields["arm_target_active"]
        or fields["pickup_busy"]
        or fields["object_tracking_active"]
        or fields["object_tracking_busy"]
        or fields["object_search_active"]
        or fields["object_search_busy"]
        or fields["arm_stack_busy"]
        or node.control_mode not in ("JOINT", "CARTESIAN", "CYLINDRICAL")
    )
    arm_target_html = f"""
    <div class="card" id="arm_target_card">
      <h2>🧭 Arm target test</h2>
      <p>Status: <span id="st_arm_target_state">{fields["arm_target_state"]}</span>
      · mode <span class="mono" id="st_arm_target_mode">{fields["arm_target_mode"]}</span></p>
      <p class="mono" id="st_arm_target_message">{fields["arm_target_message"]}</p>
      <label class="small" for="arm_target_mode">Absolute target type</label>
      <form method="POST" action="/arm_target"
            onsubmit="return confirm('Move the arm to this absolute target? Make sure the entire arm workspace is clear, remote arm control is enabled, and the joystick is released.')">
        <input type="hidden" name="csrf" value="{html.escape(node.csrf_token, quote=True)}">
        <select class="targetmode" id="arm_target_mode" name="mode">
          <option value="JOINT"{" selected" if initial_target_mode == "JOINT" else ""}>JOINT target</option>
          <option value="CARTESIAN"{" selected" if initial_target_mode == "CARTESIAN" else ""}>CARTESIAN target</option>
          <option value="CYLINDRICAL"{" selected" if initial_target_mode == "CYLINDRICAL" else ""}>CYLINDRICAL target</option>
        </select>
        <div class="targetgrid">{target_inputs}</div>
        <div class="btnrow">
          <button id="send_arm_target_btn" type="submit"{
              " disabled" if target_send_blocked else ""
          }>▶ Run target</button>
          <button class="ghost" id="arm_target_use_current" type="button">↙ Use current position</button>
        </div>
      </form>
      <form class="inline" method="POST" action="/arm_target_stop">
        <input type="hidden" name="csrf" value="{html.escape(node.csrf_token, quote=True)}">
        <button class="stop" id="stop_arm_target_btn" type="submit">■ Stop target</button>
      </form>
      <p class="small">Selected-mode feedback: <span class="mono"
      id="arm_target_current_text">{html.escape(current_target_text)}</span></p>
      <p class="small">All targets are absolute. JOINT uses radians.
      CARTESIAN uses XYZ in meters + RPY in radians in the world frame.
      CYLINDRICAL uses radius in meters, theta in radians, world Z, then RPY.
      The controller rejects targets outside workspace/joint limits,
      unreachable IK, wrist flips, and self-collision paths. Stop target holds
      the current joint feedback; it is not a hardware emergency stop.</p>
    </div>
    """

    restart = node.arm_restart_snapshot()
    restart_disabled = " disabled" if restart["phase"] == "restarting" else ""
    restart_label = (
        "⏳ Restarting OM6DOF…"
        if restart["phase"] == "restarting"
        else "♻ Restart OM6DOF stack"
    )
    arm_service_html = f"""
    <div class="card">
      <h2>🦾 OM6DOF hardware/controller service</h2>
      <p>Status: <span id="arm_stack_card">{fields["arm_stack"]}</span></p>
      <form method="POST" action="/restart_om6dof"
            onsubmit="return confirm('Restart the entire OM6DOF stack? Arm control will be temporarily interrupted, and the arm may move during initialization.')">
        <input type="hidden" name="csrf" value="{html.escape(node.csrf_token, quote=True)}">
        <button class="stop" id="restart_om6dof_btn" type="submit"{restart_disabled}>{restart_label}</button>
      </form>
      <p class="small">Last restart result:
      <span id="st_arm_restart_message">{fields["arm_restart_message"]}</span></p>
      <p class="small">Restarts the single <span class="mono">{OM6DOF_SERVICE}</span>
      unit: ros2_control bringup, om6dof_controller, and teleop. Use only when
      the arm's motion area is clear.</p>
    </div>
    """

    perception_active = fields["perception_active"]
    perception_html = f"""
    <div class="card">
      <h2>👁️ OM6DOF perception</h2>
      <p>Status: <span id="perception_card">{fields["perception"]}</span></p>
      <p>Tracking: <span class="mono" id="st_perception_tracking">{fields["perception_tracking"]}</span></p>
      <p>Object ↔ EoE distance:
      <span id="st_perception_distance">{fields["perception_distance"]}</span></p>
      <form method="POST" action="/run_perception_pick"
            onsubmit="return confirm('Run visual-servo pickup? The arm will pan–tilt track, approach in stages, lock left–right, move forward, then grasp. Make sure the area is clear and remote F3 is OFF.')">
        <input type="hidden" name="csrf" value="{html.escape(node.csrf_token, quote=True)}">
        <button id="pickup_object_btn" type="submit"{
            " disabled" if fields["pickup_busy"] or fields["object_search_active"] or not perception_active else ""
        }>🤖 Pickup object</button>
      </form>
      <p class="small">Pickup result:
      <span id="st_pickup_message">{fields["pickup_message"]}</span></p>
      <p>Object tracking pan–tilt:
      <span class="mono" id="st_object_tracking_message">{fields["object_tracking_message"]}</span></p>
      <div class="btnrow">
        <form class="inline" method="POST" action="/start_object_tracking"
              onsubmit="return confirm('The arm camera will follow the object left/right and up/down using joint1 + joint5. Is the area clear?')">
          <input type="hidden" name="csrf" value="{html.escape(node.csrf_token, quote=True)}">
          <button id="start_object_tracking_btn" type="submit"{
              " disabled" if fields["object_tracking_active"] or fields["object_tracking_busy"] or fields["object_search_active"] or not perception_active else ""
          }>🎯 Start tracking pan–tilt</button>
        </form>
        <form class="inline" method="POST" action="/stop_object_tracking">
          <input type="hidden" name="csrf" value="{html.escape(node.csrf_token, quote=True)}">
          <button class="stop" id="stop_object_tracking_btn" type="submit"{
              " disabled" if not fields["object_tracking_active"] or fields["object_tracking_busy"] else ""
          }>■ Stop tracking</button>
        </form>
      </div>
      <p>Arm searching state:
      <span class="mono" id="st_object_search_message">{fields["object_search_message"]}</span></p>
      <div class="btnrow">
        <form class="inline" method="POST" action="/start_object_search"
              onsubmit="return confirm('The arm will move to a safe pose, then sweep joint1 center/left/right at low, medium, and high angles. Make sure the area is clear and remote F3 is OFF.')">
          <input type="hidden" name="csrf" value="{html.escape(node.csrf_token, quote=True)}">
          <button id="start_object_search_btn" type="submit"{
              " disabled" if fields["object_search_active"] or fields["object_search_busy"] or fields["pickup_busy"] or fields["object_tracking_active"] or not perception_active else ""
          }>🔎 Start searching state</button>
        </form>
        <form class="inline" method="POST" action="/stop_object_search">
          <input type="hidden" name="csrf" value="{html.escape(node.csrf_token, quote=True)}">
          <button class="stop" id="stop_object_search_btn" type="submit"{
              " disabled" if not fields["object_search_active"] or fields["object_search_busy"] else ""
          }>■ Stop searching</button>
        </form>
      </div>
      <div class="btnrow">
        <form class="inline" method="POST" action="/start_perception">
          <input type="hidden" name="csrf" value="{html.escape(node.csrf_token, quote=True)}">
          <button id="start_perception_btn" type="submit"{" disabled" if perception_active else ""}>▶ Start perception</button>
        </form>
        <form class="inline" method="POST" action="/stop_perception">
          <input type="hidden" name="csrf" value="{html.escape(node.csrf_token, quote=True)}">
          <button class="stop" id="stop_perception_btn" type="submit"{"" if perception_active else " disabled"}>■ Stop perception</button>
        </form>
      </div>
      <form method="POST" action="/target_perception">
        <input type="hidden" name="csrf" value="{html.escape(node.csrf_token, quote=True)}">
        <div class="chatrow">
          <input name="target" maxlength="200" placeholder="COCO target, e.g. bottle, cup, person" required>
          <button type="submit">Set target</button>
        </div>
      </form>
      <p class="small">Perception runs on the robot/NX and opens the RealSense
      camera. The target must be a COCO class supported by YOLOX. Search sweeps
      low → medium → high and reports FOUND only after detecting the target in
      10 consecutive frames. Pickup uses pan–tilt tracking while approaching the
      object directly along the 3D ray (not forced horizontal) at distances of
      16→12→8 cm, relocks left–right and up–down, then advances at close range
      and grasps. Low targets automatically use the upper surface of the 3D
      bounding box: hover above the object, then descend vertically before
      grasping. The detection overlay appears in a separate RealSense card
      without replacing the robot's built-in camera.</p>
    </div>
    """
    ddgng_active = fields["ddgng_active"]
    ddgng_html = f"""
    <div class="card">
      <h2>🕸️ OM6DOF DD-GNG YOLO</h2>
      <p>Status: <span id="ddgng_card">{fields["ddgng"]}</span></p>
      <div class="btnrow">
        <form class="inline" method="POST" action="/start_ddgng">
          <input type="hidden" name="csrf" value="{html.escape(node.csrf_token, quote=True)}">
          <button id="start_ddgng_btn" type="submit"{" disabled" if ddgng_active else ""}>▶ Start DD-GNG YOLO</button>
        </form>
        <form class="inline" method="POST" action="/stop_ddgng">
          <input type="hidden" name="csrf" value="{html.escape(node.csrf_token, quote=True)}">
          <button class="stop" id="stop_ddgng_btn" type="submit"{"" if ddgng_active else " disabled"}>■ Stop DD-GNG YOLO</button>
        </form>
      </div>
      <p class="small">Runs DD-GNG + YOLO headlessly, assigns semantic labels
      to nodes that intersect detections, and sends the RealSense overlay to the
      web UI. Systemd stops perception/pickup when DD-GNG starts so two
      processes do not open the camera at the same time.</p>
    </div>
    """

    # --- AI chat card (model selector = local Ollama models + codex + claude) ---
    # Smallest models first so the default selection is the one most likely to
    # fit the Orin GPU (7B models OOM here; 3B fits).
    ollama_models = sorted(list_ollama_models(), key=model_param_billions)
    # Robot Agent options first (they DRIVE the robot); one per local model.
    agent_opts = "".join(
        f'<option value="agent:{html.escape(m)}">🦿 Robot Agent · {html.escape(m)}</option>'
        for m in ollama_models
    )
    if ollama_models:
        opts = agent_opts + "".join(
            f'<option value="ollama:{html.escape(m)}">Chat · {html.escape(m)}</option>'
            for m in ollama_models
        )
    else:
        opts = '<option value="" disabled>(no local Ollama models)</option>'
    opts += ('<option value="codex">Codex CLI</option>'
             '<option value="claude">Claude Code CLI</option>')
    chat_html = f"""
    <div class="card chatcard">
      <h2>🦿 Robot Agent — command via chat</h2>
      <div class="chatlog" id="chatlog">
        <div class="msg ai hint">Type a command, e.g. <b>move forward 1 meter</b>,
        <b>turn left</b>, <b>stop</b>, <b>battery level</b>,
        <b>list ros topics</b>.</div>
      </div>
      <div class="chatrow">
        <select id="chatmodel" title="Choose the AI backend">{opts}</select>
        <input id="chatinput" type="text" placeholder="Robot command… e.g. 'move forward 1 meter'" autocomplete="off">
        <button id="chatsend" type="button">Send</button>
        <form class="inline" method="POST" action="/stop_llm"
              title="Unload the LLM currently held in GPU/RAM">
          <button class="stop" type="submit">⛔ Kill LLM</button>
        </form>
      </div>
      <p class="small" id="chatstatus"></p>
      <p class="small">🦿 <b>Robot Agent</b> = the local LLM turns your message into a
      real action (move, teleop, ask battery/topics/nodes). <b>Chat</b> = plain Q&amp;A.
      <b>Kill LLM</b> frees GPU/RAM by unloading whichever model is loaded (see the
      "LLM loaded" row below). Skill: skills/go2w_control_skill.md.</p>
    </div>
    """

    # one-shot flash banner (kill result etc.)
    flash_html = ""
    if node.flash:
        flash_html = f'<div class="flash">{html.escape(node.flash)}</div>'
        node.flash = ""

    now = time.strftime("%Y-%m-%d %H:%M:%S")
    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Go2W Monitor — {html.escape(info['hostname'])}</title>
<style>{CSS}</style></head><body>
<header><h1>🤖 Go2W Robot Monitor</h1>
<div class="header-actions">
<time class="header-clock" id="header_clock" aria-label="Local time">--:--</time>
<div class="header-ram" id="st_ram">{fields["ram"]}</div>
<div class="header-battery" id="st_battery">{fields["battery"]}</div>
<a class="btn ghost" href="/">↻ Refresh</a>
<form class="inline" method="POST" action="/restart"
      onsubmit="return confirm('Restart the web monitor service? The page will reconnect in a few seconds.')">
  <button class="stop" type="submit">♻ Restart monitor</button>
</form>
<form class="inline" method="POST" action="/shutdown"
      onsubmit="return confirm('Shut down the web monitor? It will NOT start again until manually started with systemctl start.')">
  <button class="kill" type="submit">🛑 Kill monitor</button>
</form>
</div></header>
<div id="action_notice" class="actionnotice" aria-live="polite"></div>
<main>
<p class="small">Status refreshes automatically every 1 s; control buttons
run without reloading this page. Node &amp; topic lists are available via chat.
Snapshot {now}.</p>
{flash_html}
{dup_banner}
{chat_html}
<div class="grid">
  <div>
    <div class="card"><h2>📊 Robot status <span class="small" id="st_dot">●</span></h2>
      <table>{status_rows}</table></div>
    {arm_service_html}
    {perception_html}
    {ddgng_html}
    {teleop_html}
    {arm_target_html}
  </div>
  <div>
    {cam_html}
    {perception_cam_html}
    {ddgng_cam_html}
    {audio_html}
  </div>
</div>
</main>
{SCRIPTS}
</body></html>"""


# --------------------------------------------------------------------------- #
#  HTTP handler                                                               #
# --------------------------------------------------------------------------- #
def make_handler(node: MonitorNode, cam: ForwardedImageStream, audio=None):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):  # silence per-request logging
            pass

        def _send_html(self, body: str, code: int = 200):
            data = body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header(
                "Content-Security-Policy",
                "frame-ancestors 'none'; form-action 'self'",
            )
            self.send_header("Referrer-Policy", "no-referrer")
            self.end_headers()
            self.wfile.write(data)

        def _send_json(self, obj, code: int = 200):
            data = json.dumps(obj).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Frame-Options", "DENY")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path.startswith("/ddgng.mjpg"):
                return self._stream_camera(
                    getattr(node, "ddgng_camera", None), "DD-GNG"
                )
            if self.path.startswith("/perception.mjpg"):
                return self._stream_camera(
                    getattr(node, "perception_camera", None), "perception"
                )
            if self.path.startswith("/camera.mjpg"):
                return self._stream_camera(cam, "built-in camera")
            if self.path.startswith("/audio.pcm"):
                return self._stream_audio()
            if self.path.startswith("/status.json"):
                try:
                    return self._send_json(status_fields(node, cam, audio))
                except Exception as exc:
                    return self._send_json({"error": str(exc)}, 500)
            if self.path in ("/", "/index.html"):
                try:
                    return self._send_html(render_page(node, cam, audio))
                except Exception as exc:
                    return self._send_html(
                        f"<pre>render error: {html.escape(str(exc))}</pre>", 500
                    )
            self.send_error(404)

        def _read_form(self, max_bytes: int = MAX_FORM_BODY_BYTES) -> dict:
            try:
                length = int(self.headers.get("Content-Length", 0) or 0)
            except (TypeError, ValueError) as exc:
                raise ValueError("invalid Content-Length") from exc
            if length < 0:
                raise ValueError("invalid Content-Length")
            if length > max_bytes:
                # Do not consume an attacker-controlled body. Closing the
                # keep-alive connection prevents it becoming the next request.
                self.close_connection = True
                raise ValueError(f"form body exceeds {max_bytes} bytes")
            body = self.rfile.read(length).decode("utf-8") if length else ""
            return {k: v[0] for k, v in parse_qs(body).items()}

        def _redirect_home(self):
            if self.headers.get("X-Requested-With", "").lower() == "fetch":
                message = str(node.flash or "Request accepted.")
                node.flash = ""
                lowered = message.lower()
                error_markers = (
                    "failed", "denied", "rejected", "invalid", "unavailable",
                    "timed out", "not ready", "disable remote",
                    "still being processed", "already running",
                )
                ok = not any(marker in lowered for marker in error_markers)
                return self._send_json({
                    "ok": ok,
                    "message": message,
                    "refresh_ms": 250,
                }, 200 if ok else 409)
            self.send_response(303)
            self.send_header("Location", "/")
            self.send_header("Content-Length", "0")
            self.send_header("X-Frame-Options", "DENY")
            self.end_headers()

        def do_POST(self):
            if self.path in ("/start_ddgng", "/stop_ddgng"):
                try:
                    form = self._read_form()
                except (UnicodeError, ValueError) as exc:
                    return self._send_html(
                        "<h2>Request rejected</h2>"
                        f"<p>{html.escape(str(exc))}</p>",
                        413 if "exceeds" in str(exc) else 400,
                    )
                if not csrf_token_matches(form.get("csrf", ""), node.csrf_token):
                    node.get_logger().warn(
                        "Rejected DD-GNG control with invalid CSRF token."
                    )
                    return self._send_html(
                        "<h2>403 Forbidden</h2><p>Invalid control token.</p>",
                        403,
                    )
                action = "start" if self.path == "/start_ddgng" else "stop"
                ssh_host = getattr(node, "robot_ssh_host", "")
                try:
                    completed = invoke_ddgng_service(action, ssh_host)
                except (OSError, subprocess.SubprocessError) as exc:
                    node.flash = f"DD-GNG {action} failed: {exc}"
                else:
                    detail = (completed.stderr or completed.stdout).strip()
                    if completed.returncode == 0:
                        node.flash = f"DD-GNG {action} requested."
                    else:
                        node.flash = (
                            f"DD-GNG {action} denied: "
                            f"{detail or 'unknown error'}. Install the "
                            "om6dof-dd-gng user service."
                        )
                node.get_logger().info(node.flash)
                return self._redirect_home()
            if self.path == "/run_perception_pick":
                try:
                    form = self._read_form()
                except (UnicodeError, ValueError) as exc:
                    return self._send_html(
                        "<h2>Request rejected</h2>"
                        f"<p>{html.escape(str(exc))}</p>",
                        413 if "exceeds" in str(exc) else 400,
                    )
                if not csrf_token_matches(form.get("csrf", ""), node.csrf_token):
                    node.get_logger().warn(
                        "Rejected perception pickup with invalid CSRF token."
                    )
                    return self._send_html(
                        "<h2>403 Forbidden</h2><p>Invalid control token.</p>",
                        403,
                    )
                started, message = node.request_perception_pick()
                node.flash = message
                if started:
                    node.get_logger().warn(
                        "Perception pickup requested from web monitor."
                    )
                else:
                    node.get_logger().info(message)
                return self._redirect_home()
            if self.path in (
                "/start_object_tracking", "/stop_object_tracking"
            ):
                try:
                    form = self._read_form()
                except (UnicodeError, ValueError) as exc:
                    return self._send_html(
                        "<h2>Request rejected</h2>"
                        f"<p>{html.escape(str(exc))}</p>",
                        413 if "exceeds" in str(exc) else 400,
                    )
                if not csrf_token_matches(form.get("csrf", ""), node.csrf_token):
                    node.get_logger().warn(
                        "Rejected object tracking with invalid CSRF token.")
                    return self._send_html(
                        "<h2>403 Forbidden</h2><p>Invalid control token.</p>",
                        403,
                    )
                enable = self.path == "/start_object_tracking"
                started, message = node.request_object_tracking(enable)
                node.flash = message
                if started:
                    node.get_logger().warn(
                        f"Object tracking {'start' if enable else 'stop'} "
                        "requested from web monitor.")
                else:
                    node.get_logger().info(message)
                return self._redirect_home()
            if self.path in (
                "/start_object_search", "/stop_object_search"
            ):
                try:
                    form = self._read_form()
                except (UnicodeError, ValueError) as exc:
                    return self._send_html(
                        "<h2>Request rejected</h2>"
                        f"<p>{html.escape(str(exc))}</p>",
                        413 if "exceeds" in str(exc) else 400,
                    )
                if not csrf_token_matches(form.get("csrf", ""), node.csrf_token):
                    node.get_logger().warn(
                        "Rejected object search with invalid CSRF token.")
                    return self._send_html(
                        "<h2>403 Forbidden</h2><p>Invalid control token.</p>",
                        403,
                    )
                enable = self.path == "/start_object_search"
                started, message = node.request_object_search(enable)
                node.flash = message
                if started:
                    node.get_logger().warn(
                        f"Object search {'start' if enable else 'stop'} "
                        "requested from web monitor.")
                else:
                    node.get_logger().info(message)
                return self._redirect_home()
            if self.path in (
                "/start_perception", "/stop_perception", "/target_perception"
            ):
                try:
                    form = self._read_form()
                except (UnicodeError, ValueError) as exc:
                    return self._send_html(
                        "<h2>Request rejected</h2>"
                        f"<p>{html.escape(str(exc))}</p>",
                        413 if "exceeds" in str(exc) else 400,
                    )
                if not csrf_token_matches(form.get("csrf", ""), node.csrf_token):
                    node.get_logger().warn(
                        "Rejected perception control with invalid CSRF token."
                    )
                    return self._send_html(
                        "<h2>403 Forbidden</h2><p>Invalid control token.</p>",
                        403,
                    )
                if self.path == "/target_perception":
                    target = form.get("target", "").strip()
                    if not target or len(target) > 200:
                        return self._send_html(
                            "<h2>Invalid target</h2>", 400
                        )
                    node.set_perception_target(target)
                    node.flash = f"Perception target set: {target}"
                    node.get_logger().info(node.flash)
                    return self._redirect_home()
                action = (
                    "start" if self.path == "/start_perception" else "stop"
                )
                ssh_host = getattr(node, "robot_ssh_host", "")
                try:
                    completed = invoke_perception_service(action, ssh_host)
                except (OSError, subprocess.SubprocessError) as exc:
                    node.flash = f"Perception {action} failed: {exc}"
                else:
                    detail = (completed.stderr or completed.stdout).strip()
                    if completed.returncode == 0:
                        node.flash = f"Perception {action} requested."
                    else:
                        node.flash = (
                            f"Perception {action} denied: {detail or 'unknown error'}. "
                            "Install and start the perception user service on the robot."
                        )
                node.get_logger().info(node.flash)
                return self._redirect_home()
            if self.path in ("/arm_target", "/arm_target_stop"):
                try:
                    form = self._read_form()
                except (UnicodeError, ValueError) as exc:
                    return self._send_html(
                        "<h2>Request rejected</h2>"
                        f"<p>{html.escape(str(exc))}</p>",
                        413 if "exceeds" in str(exc) else 400,
                    )
                if not csrf_token_matches(form.get("csrf", ""), node.csrf_token):
                    node.get_logger().warn(
                        "Rejected arm target with invalid CSRF token."
                    )
                    return self._send_html(
                        "<h2>403 Forbidden</h2><p>Invalid control token.</p>",
                        403,
                    )
                if self.path == "/arm_target_stop":
                    started, message = node.request_arm_target_stop()
                else:
                    mode = form.get("mode", "").strip().upper()
                    try:
                        values = [
                            float(form[f"value_{index}"])
                            for index in range(1, 7)
                        ]
                    except (KeyError, TypeError, ValueError):
                        return self._send_html(
                            "<h2>Invalid arm target</h2>"
                            "<p>Enter exactly six numeric values.</p>",
                            400,
                        )
                    if not all(math.isfinite(value) for value in values):
                        return self._send_html(
                            "<h2>Invalid arm target</h2>"
                            "<p>All values must be finite.</p>",
                            400,
                        )
                    started, message = node.request_arm_target(mode, values)
                node.flash = message
                if started:
                    node.get_logger().warn(
                        f"Arm target action requested from web: {message}"
                    )
                else:
                    node.get_logger().info(message)
                return self._redirect_home()
            if self.path in (
                "/mode_joint", "/mode_cartesian", "/mode_cylindrical"
            ):
                mode = self.path.removeprefix("/mode_").upper()
                node.set_operation_mode(mode)
                node.flash = f"Operation mode request sent: {mode}."
                node.get_logger().info(node.flash)
                return self._redirect_home()
            if self.path in ("/start_teleop", "/stop_teleop", "/toggle_teleop"):
                want_start = self.path == "/start_teleop"
                want_stop = self.path == "/stop_teleop"
                running = node.teleop_running()
                try:
                    if want_start and running:
                        node.flash = "Remote arm ownership is already enabled."
                    elif want_stop and not running:
                        node.flash = "Remote arm ownership is already disabled."
                    else:
                        node.tap_f3()
                        node.flash = (
                            "F3 sent → requesting remote arm ownership."
                            if (want_start or (not running and not want_stop))
                            else "F3 sent → returning arm ownership to autonomy."
                        )
                except Exception as exc:
                    node.flash = f"teleop control failed: {exc}"
                node.get_logger().info(node.flash)
                return self._redirect_home()
            if self.path == "/restart_om6dof":
                try:
                    provided = self._read_form().get("csrf", "")
                except (UnicodeError, ValueError) as exc:
                    return self._send_html(
                        "<h2>Request rejected</h2>"
                        f"<p>{html.escape(str(exc))}</p>",
                        413 if "exceeds" in str(exc) else 400,
                    )
                if not csrf_token_matches(provided, node.csrf_token):
                    node.get_logger().warn(
                        "Rejected OM6DOF restart with invalid CSRF token."
                    )
                    return self._send_html(
                        "<h2>403 Forbidden</h2><p>Invalid restart token.</p>",
                        403,
                    )
                started, message = node.request_arm_stack_restart()
                node.flash = message
                if started:
                    node.get_logger().warn(
                        "OM6DOF full-stack restart requested from web monitor."
                    )
                else:
                    node.get_logger().info(message)
                return self._redirect_home()
            if self.path == "/chat":
                try:
                    length = int(self.headers.get("Content-Length", 0) or 0)
                    body = self.rfile.read(length).decode("utf-8") if length else "{}"
                    data = json.loads(body)
                    model = str(data.get("model", "")).strip()
                    message = str(data.get("message", "")).strip()
                except Exception as exc:
                    return self._send_json({"error": f"bad request: {exc}"}, 400)
                if not message:
                    return self._send_json({"error": "empty message"}, 400)
                if model.startswith("agent:"):
                    reply, err = route_agent(node, model.split(":", 1)[1], message)
                else:
                    reply, err = route_chat(model, message)
                return self._send_json({"reply": reply or "", "error": err})
            if self.path == "/kill_node":
                name = self._read_form().get("name", "").strip()
                if name:
                    try:
                        node.flash = kill_node(name)
                    except Exception as exc:
                        node.flash = f"kill '{name}' failed: {exc}"
                    node.get_logger().info(node.flash)
                return self._redirect_home()
            if self.path == "/restart":
                node.get_logger().info("Restart requested from web — re-execing.")
                body = (
                    "<!doctype html><meta charset='utf-8'>"
                    "<meta http-equiv='refresh' content='6; url=/'>"
                    "<title>Restarting…</title>"
                    "<body style='font-family:system-ui;background:#12141a;color:#e6e6e6;"
                    "padding:40px;text-align:center'>"
                    "<h2>♻ Restarting the web monitor…</h2>"
                    "<p>Reloading the latest code. This page reconnects automatically "
                    "in a few seconds.</p>"
                    "<p><a style='color:#7ea1ff' href='/'>Click here</a> if it doesn't.</p>"
                    "</body>"
                )
                self._send_html(body)
                try:
                    self.wfile.flush()
                except Exception:
                    pass
                restart_self()
                return
            if self.path == "/shutdown":
                node.get_logger().info(
                    "Shutdown requested from web — exiting.")
                body = (
                    "<!doctype html><meta charset='utf-8'>"
                    "<title>Shutting down…</title>"
                    "<body style='font-family:system-ui;background:#12141a;"
                    "color:#e6e6e6;padding:40px;text-align:center'>"
                    "<h2>🛑 Web monitor shut down</h2>"
                    "<p>The monitor will remain off until manually started:</p>"
                    "<pre style='color:#7ea1ff'>sudo systemctl start "
                    "om6dof-web-monitor.service</pre></body>"
                )
                self._send_html(body)
                try:
                    self.wfile.flush()
                except Exception:
                    pass
                shutdown_self(cam)
                return
            if self.path == "/preload_llm":
                try:
                    length = int(self.headers.get("Content-Length", 0) or 0)
                    body = self.rfile.read(length).decode("utf-8") if length else "{}"
                    value = str(json.loads(body).get("model", "")).strip()
                except Exception as exc:
                    return self._send_json({"error": f"bad request: {exc}"}, 400)
                model = raw_ollama_model(value)
                if not model:
                    return self._send_json({"ok": False, "skipped": True})
                ok, info = preload_ollama(model)
                return self._send_json({"ok": ok, "model": info})
            if self.path == "/stop_llm":
                # An optional `model` form field kills just that one; otherwise
                # unload every currently-loaded model.
                model = self._read_form().get("model", "").strip()
                try:
                    if model:
                        ok, info = stop_ollama(model)
                        node.flash = (f"Killed LLM {info}." if ok
                                      else f"Failed to kill LLM {info}.")
                    else:
                        node.flash = stop_running_ollama()
                except Exception as exc:
                    node.flash = f"stop LLM failed: {exc}"
                node.get_logger().info(node.flash)
                return self._redirect_home()
            self.send_error(404)

        def _stream_camera(self, stream, label="camera"):
            if stream is None or not stream.available():
                return self._send_html(
                    f"<pre>no {html.escape(label)} stream</pre>", 503
                )
            self.send_response(200)
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                seq = -1
                while stream.available():
                    frame, seq = stream.next_frame(seq)
                    if frame is None:
                        continue
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        f"Content-Length: {len(frame)}\r\n\r\n".encode()
                    )
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _stream_audio(self):
            if audio is None or not audio.available():
                return self._send_html("<pre>no live microphone stream</pre>", 503)
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Audio-Format", "pcm_s16le")
            self.send_header("X-Audio-Sample-Rate", str(audio.SAMPLE_RATE))
            self.send_header("X-Audio-Channels", str(audio.CHANNELS))
            self.end_headers()
            try:
                seq = -1
                while audio.available():
                    chunk, seq = audio.next_chunk(seq)
                    if not chunk:
                        continue
                    self.wfile.write(chunk)
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


# --------------------------------------------------------------------------- #
#  main                                                                        #
# --------------------------------------------------------------------------- #
def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    node.declare_parameter("port", 8080)
    port = int(node.get_parameter("port").value)
    cam = node.camera
    audio = node.audio

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    # give discovery a moment to populate the graph before first request
    time.sleep(1.5)

    httpd = ThreadingHTTPServer(
        ("0.0.0.0", port), make_handler(node, cam, audio))
    ip = subprocess.run(
        ["hostname", "-I"], capture_output=True, text=True
    ).stdout.split()
    url = f"http://{ip[0]}:{port}" if ip else f"http://<robot-ip>:{port}"
    node.get_logger().info(f"Go2W web monitor serving at {url}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
