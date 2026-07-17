"""Send final microphone transcripts to the local Ollama LLM on the AGX."""

from __future__ import annotations

import json
import queue
import re
import threading
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


SYSTEM_PROMPT = (
    "You are a concise voice assistant embedded in a Unitree Go2W robot. "
    "The user's text comes from speech recognition and may contain minor "
    "mistakes, so infer the intended wording. Reply briefly in English. "
    "This interface is conversational only; do not claim that you executed "
    "a physical robot action."
)


def extract_wake_command(text: str, wake_word: str) -> str:
    """Return a voice command only when explicitly addressed to the robot.

    A plain ``stop`` remains available without the wake word as a safe override.
    """
    cleaned = " ".join(text.strip().split())
    if re.fullmatch(r"stop[.!?]*", cleaned, flags=re.IGNORECASE):
        return "stop"
    pattern = (
        rf"^(?:hey\s+)?{re.escape(wake_word)}s?\b"
        rf"[\s,.:;!\-]*(.+)$"
    )
    match = re.match(pattern, cleaned, flags=re.IGNORECASE)
    return match.group(1).strip() if match else ""


def build_payload(model: str, history: list[dict]) -> bytes:
    return json.dumps({
        "model": model,
        "messages": [{"role": "system", "content": SYSTEM_PROMPT}, *history],
        "stream": False,
        "keep_alive": "30m",
        "options": {
            "num_ctx": 4096,
            "num_predict": 192,
            "temperature": 0.2,
        },
    }).encode("utf-8")


def extract_reply(payload: dict) -> str:
    if not isinstance(payload, dict):
        return ""
    message = payload.get("message", {})
    if not isinstance(message, dict):
        return ""
    return " ".join(str(message.get("content", "")).strip().split())


def extract_agent_reply(payload: dict) -> str:
    if not isinstance(payload, dict):
        raise ValueError("invalid Robot Agent response")
    error = str(payload.get("error") or "").strip()
    if error:
        raise ValueError(error)
    reply = " ".join(str(payload.get("reply", "")).strip().split())
    if not reply:
        raise ValueError("Robot Agent returned an empty response")
    return reply


def robot_agent_chat(
        url: str, model: str, text: str, timeout: float) -> str:
    body = json.dumps({
        "model": f"agent:{model}",
        "message": text,
    }).encode("utf-8")
    request = urllib.request.Request(
        url.rstrip("/") + "/chat",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return extract_agent_reply(
            json.loads(response.read().decode("utf-8")))


def ollama_chat(
        url: str, model: str, history: list[dict], timeout: float) -> str:
    request = urllib.request.Request(
        url.rstrip("/") + "/api/chat",
        data=build_payload(model, history),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        reply = extract_reply(json.loads(response.read().decode("utf-8")))
    if not reply:
        raise ValueError("Ollama returned an empty response")
    return reply


class SttLlmBridge(Node):
    def __init__(self) -> None:
        super().__init__("stt_llm_bridge")
        self.declare_parameter("stt_topic", "/application/stt/event")
        self.declare_parameter("response_topic", "/application/llm/response")
        self.declare_parameter("response_event_topic", "/application/llm/event")
        self.declare_parameter("status_topic", "/application/llm/status")
        self.declare_parameter("ollama_url", "http://127.0.0.1:11434")
        self.declare_parameter("robot_agent_url", "http://127.0.0.1:8080")
        self.declare_parameter("execute_actions", True)
        self.declare_parameter("wake_word", "robot")
        self.declare_parameter(
            "model", "qwen3-vl:8b-instruct-q4_K_M")
        self.declare_parameter("request_timeout_sec", 180.0)
        self.declare_parameter("history_turns", 3)

        stt_topic = str(self.get_parameter("stt_topic").value)
        response_topic = str(self.get_parameter("response_topic").value)
        response_event_topic = str(
            self.get_parameter("response_event_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        self.ollama_url = str(self.get_parameter("ollama_url").value)
        self.robot_agent_url = str(
            self.get_parameter("robot_agent_url").value)
        self.execute_actions = bool(
            self.get_parameter("execute_actions").value)
        self.wake_word = str(
            self.get_parameter("wake_word").value).strip() or "robot"
        self.model = str(self.get_parameter("model").value)
        self.timeout = float(
            self.get_parameter("request_timeout_sec").value)
        self.history_messages = max(
            2, int(self.get_parameter("history_turns").value) * 2)

        event_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.response_pub = self.create_publisher(
            String, response_topic, state_qos)
        self.response_event_pub = self.create_publisher(
            String, response_event_topic, event_qos)
        self.status_pub = self.create_publisher(String, status_topic, state_qos)
        self.create_subscription(String, stt_topic, self._on_transcript, event_qos)

        self.jobs: queue.Queue[str | None] = queue.Queue(maxsize=3)
        self.history: list[dict] = []
        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()
        self._publish_status("ready")
        self.get_logger().info(
            f"STT -> {'Robot Agent ACTION' if self.execute_actions else 'LLM'}: "
            f"{stt_topic} -> {self.model} -> {response_topic}")

    def _on_transcript(self, message: String) -> None:
        text = message.data.strip()
        if not text:
            return
        if self.execute_actions:
            command = extract_wake_command(text, self.wake_word)
            if not command:
                self.get_logger().info(
                    f"Ignored transcript without wake word '{self.wake_word}': "
                    f"{text}")
                self._publish_status("waiting_wake_word")
                return
            text = command
        try:
            self.jobs.put_nowait(text)
            self._publish_status("queued")
        except queue.Full:
            self.get_logger().warning("LLM queue full; dropping transcript")
            self._publish_status("busy")

    def _worker(self) -> None:
        while True:
            text = self.jobs.get()
            if text is None:
                return
            self._publish_status("thinking")
            request_history = [
                *self.history,
                {"role": "user", "content": text},
            ]
            try:
                if self.execute_actions:
                    reply = robot_agent_chat(
                        self.robot_agent_url, self.model, text, self.timeout)
                else:
                    reply = ollama_chat(
                        self.ollama_url, self.model,
                        request_history, self.timeout)
                    self.history.extend([
                        {"role": "user", "content": text},
                        {"role": "assistant", "content": reply},
                    ])
                    self.history = self.history[-self.history_messages:]
                response = String(data=reply)
                self.response_pub.publish(response)
                # TTS consumes a volatile event so restarting it never repeats
                # a stale, transient-local answer.
                self.response_event_pub.publish(response)
                self.get_logger().info(f"Agent response: {reply}")
                self._publish_status("ready")
            except (OSError, ValueError, json.JSONDecodeError,
                    urllib.error.URLError) as exc:
                self.get_logger().error(f"Agent request failed: {exc}")
                self._publish_status("error")
            finally:
                self.jobs.task_done()

    def _publish_status(self, status: str) -> None:
        self.status_pub.publish(String(data=status))

    def destroy_node(self):
        try:
            self.jobs.put_nowait(None)
        except queue.Full:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SttLlmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
