"""Launch the selected local or Go2W microphone bridge."""

from __future__ import annotations

import os
from pathlib import Path

from application_web_monitor.audio_control import load_audio_config


UNITREE_DDS = (
    "<CycloneDDS><Domain><General><Interfaces>"
    '<NetworkInterface name="eno1" priority="default" multicast="default" />'
    "</Interfaces></General></Domain></CycloneDDS>"
)


def main() -> None:
    config = load_audio_config()
    selected = str(config.get("input", ""))
    if selected == "go2w:microphone":
        carrier = Path("/sys/class/net/eno1/carrier")
        if not carrier.exists() or carrier.read_text().strip() != "1":
            raise SystemExit("Go2W microphone unavailable: eno1 has no carrier")
        os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
        os.environ["CYCLONEDDS_URI"] = UNITREE_DDS
        command = ["ros2", "run", "application_web_monitor", "unitree_audio_bridge"]
    elif selected.startswith("pulse:"):
        os.environ.pop("CYCLONEDDS_URI", None)
        source = selected.removeprefix("pulse:")
        command = [
            "ros2", "run", "application_web_monitor", "dji_audio_bridge",
            "--ros-args", "-p", f"pulse_source:={source}",
        ]
    else:
        raise SystemExit("No valid microphone is configured")
    os.execvp(command[0], command)


if __name__ == "__main__":
    main()
