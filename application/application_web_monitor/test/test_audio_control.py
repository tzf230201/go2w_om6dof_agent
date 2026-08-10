import json
from types import SimpleNamespace

import application_web_monitor.audio_control as audio_control


def test_parse_pactl_short_excludes_monitor_sources():
    output = (
        "0\talsa_output.card.monitor\tmodule\ts16le 2ch 48000Hz\tSUSPENDED\n"
        "1\talsa_input.usb-DJI\tmodule\ts16le 2ch 48000Hz\tRUNNING\n"
    )
    assert audio_control.parse_pactl_short(output) == [{
        "id": "pulse:alsa_input.usb-DJI",
        "label": "alsa_input.usb-DJI",
        "detail": "s16le 2ch 48000Hz",
    }]


def test_save_audio_config_only_accepts_discovered_devices(monkeypatch, tmp_path):
    config_path = tmp_path / "audio.json"
    monkeypatch.setattr(audio_control, "AUDIO_CONFIG_PATH", config_path)
    monkeypatch.setattr(audio_control, "available_audio_devices", lambda: {
        "inputs": [{"id": "pulse:mic"}],
        "outputs": [{"id": "pulse:speaker"}],
        "go2w_connected": False,
    })

    saved = audio_control.save_audio_config("pulse:mic", "pulse:speaker")

    assert saved == {"input": "pulse:mic", "output": "pulse:speaker"}
    assert json.loads(config_path.read_text()) == saved


def test_start_pipeline_stops_partial_start_on_failure(monkeypatch):
    calls = []
    monkeypatch.setattr(audio_control, "save_audio_config", lambda *_args: {})

    def fake_run(argv, _timeout=5.0):
        calls.append(argv)
        failed = "start" in argv
        return SimpleNamespace(
            returncode=1 if failed else 0,
            stdout="",
            stderr="start failed" if failed else "",
        )

    monkeypatch.setattr(audio_control, "_run", fake_run)

    ok, message = audio_control.control_audio_pipeline(
        "start", "pulse:mic", "pulse:speaker"
    )

    assert ok is False
    assert message == "start failed"
    assert any("stop" in call for call in calls)
