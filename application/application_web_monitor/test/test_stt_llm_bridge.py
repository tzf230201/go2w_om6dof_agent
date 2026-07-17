import json

import pytest

from application_web_monitor.stt_llm_bridge import (
    build_payload,
    extract_agent_reply,
    extract_reply,
    extract_wake_command,
)


def test_build_payload_contains_model_system_and_history():
    body = json.loads(build_payload(
        "test-model", [{"role": "user", "content": "Hello"}]))
    assert body["model"] == "test-model"
    assert body["messages"][0]["role"] == "system"
    assert body["messages"][1] == {"role": "user", "content": "Hello"}
    assert body["stream"] is False


def test_extract_reply_normalizes_whitespace():
    assert extract_reply({
        "message": {"content": "  Hello,\n  operator.  "},
    }) == "Hello, operator."
    assert extract_reply({"message": {}}) == ""


def test_extract_agent_reply_checks_error_and_normalizes():
    assert extract_agent_reply({"reply": "  Status is ready.\n", "error": None}) \
        == "Status is ready."
    with pytest.raises(ValueError, match="unsafe request"):
        extract_agent_reply({"reply": "", "error": "unsafe request"})


def test_extract_wake_command_requires_robot_but_always_allows_stop():
    assert extract_wake_command("Robot, move forward", "robot") \
        == "move forward"
    assert extract_wake_command("Hey robot turn left", "robot") \
        == "turn left"
    assert extract_wake_command("Robots, stand up", "robot") == "stand up"
    assert extract_wake_command("Hey robots turn right", "robot") \
        == "turn right"
    assert extract_wake_command("move forward", "robot") == ""
    assert extract_wake_command("Subtitles by Amara.org", "robot") == ""
    assert extract_wake_command("STOP!", "robot") == "stop"
