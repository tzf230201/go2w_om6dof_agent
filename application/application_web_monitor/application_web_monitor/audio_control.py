"""On-demand audio device discovery and service control for the dashboard."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
from typing import Iterable


AUDIO_CONFIG_PATH = Path.home() / ".config" / "om6dof" / "audio.json"
AUDIO_UNITS = (
    "application-audio-bridge.service",
    "application-stt.service",
    "application-stt-llm.service",
    "application-tts.service",
)
GO2W_INTERFACE = "eno1"


def _run(argv: list[str], timeout: float = 5.0) -> subprocess.CompletedProcess:
    return subprocess.run(
        argv, capture_output=True, text=True, timeout=timeout, check=False,
    )


def parse_pactl_short(output: str, monitor: bool = False) -> list[dict]:
    devices = []
    for line in output.splitlines():
        fields = line.split("\t")
        if len(fields) < 2:
            continue
        name = fields[1].strip()
        if not name or (not monitor and name.endswith(".monitor")):
            continue
        detail = fields[3].strip() if len(fields) > 3 else ""
        devices.append({"id": f"pulse:{name}", "label": name, "detail": detail})
    return devices


def pulse_devices(kind: str) -> list[dict]:
    if kind not in ("sources", "sinks"):
        raise ValueError("invalid PulseAudio device kind")
    try:
        result = _run(["pactl", "list", "short", kind])
    except (OSError, subprocess.SubprocessError):
        return []
    return parse_pactl_short(result.stdout) if result.returncode == 0 else []


def go2w_connected(interface: str = GO2W_INTERFACE) -> bool:
    try:
        carrier = Path(f"/sys/class/net/{interface}/carrier").read_text().strip()
        state = Path(f"/sys/class/net/{interface}/operstate").read_text().strip()
    except OSError:
        return False
    return carrier == "1" and state == "up"


def available_audio_devices() -> dict:
    inputs = pulse_devices("sources")
    outputs = pulse_devices("sinks")
    linked = go2w_connected()
    if linked:
        inputs.append({
            "id": "go2w:microphone",
            "label": "Go2W built-in microphone",
            "detail": "ROS /audiosender via eno1",
        })
        outputs.append({
            "id": "go2w:speaker",
            "label": "Go2W built-in speaker",
            "detail": "Unitree audiohub via eno1",
        })
    return {"inputs": inputs, "outputs": outputs, "go2w_connected": linked}


def _validate_device(device_id: str, devices: Iterable[dict]) -> bool:
    return any(item.get("id") == device_id for item in devices)


def save_audio_config(input_id: str, output_id: str) -> dict:
    available = available_audio_devices()
    if not _validate_device(input_id, available["inputs"]):
        raise ValueError("Selected microphone is no longer available.")
    if not _validate_device(output_id, available["outputs"]):
        raise ValueError("Selected speaker is no longer available.")
    config = {"input": input_id, "output": output_id}
    AUDIO_CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    temporary = AUDIO_CONFIG_PATH.with_suffix(".tmp")
    temporary.write_text(json.dumps(config, separators=(",", ":")) + "\n")
    os.chmod(temporary, 0o600)
    os.replace(temporary, AUDIO_CONFIG_PATH)
    return config


def load_audio_config() -> dict:
    try:
        data = json.loads(AUDIO_CONFIG_PATH.read_text())
    except (OSError, ValueError, TypeError):
        return {}
    return data if isinstance(data, dict) else {}


def audio_unit_states() -> dict[str, str]:
    states = {}
    for unit in AUDIO_UNITS:
        try:
            result = _run(["systemctl", "--user", "is-active", unit], 2.0)
            states[unit] = result.stdout.strip() or "inactive"
        except (OSError, subprocess.SubprocessError):
            states[unit] = "unknown"
    return states


def control_audio_pipeline(action: str, input_id: str = "", output_id: str = "") -> tuple[bool, str]:
    if action not in ("start", "stop"):
        return False, "Unsupported audio action."
    if action == "stop":
        result = _run(["systemctl", "--user", "stop", *reversed(AUDIO_UNITS)], 15.0)
        if result.returncode:
            return False, (result.stderr.strip() or "Unable to stop audio pipeline.")
        return True, "Audio pipeline stopped."
    try:
        save_audio_config(input_id, output_id)
    except ValueError as exc:
        return False, str(exc)
    # Clear an old start-limit failure only for the four fixed, installed units.
    _run(["systemctl", "--user", "reset-failed", *AUDIO_UNITS], 5.0)
    result = _run(["systemctl", "--user", "start", *AUDIO_UNITS], 25.0)
    if result.returncode:
        _run(["systemctl", "--user", "stop", *reversed(AUDIO_UNITS)], 15.0)
        return False, (result.stderr.strip() or "Unable to start audio pipeline.")
    return True, "Audio pipeline started with the selected microphone and speaker."
