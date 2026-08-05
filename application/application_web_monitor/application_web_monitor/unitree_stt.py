"""Turn the filtered Unitree microphone stream into English text.

The audio bridge performs Opus decoding, noise suppression, and VAD. This node
groups the speech-only 48 kHz PCM frames into utterances, converts them to a
16 kHz WAV, and sends them to the local whisper.cpp HTTP server on the AGX.
"""

from __future__ import annotations

import audioop
import io
import json
import queue
import threading
import urllib.error
import urllib.request
import uuid
import wave

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String, UInt8MultiArray


INPUT_RATE = 48000
WHISPER_RATE = 16000
CHANNELS = 1
SAMPLE_WIDTH = 2


def pcm_to_wav(pcm: bytes) -> bytes:
    """Resample mono PCM S16LE from 48 kHz and wrap it as a 16 kHz WAV."""
    resampled, _ = audioop.ratecv(
        pcm, SAMPLE_WIDTH, CHANNELS, INPUT_RATE, WHISPER_RATE, None)
    output = io.BytesIO()
    with wave.open(output, "wb") as wav:
        wav.setnchannels(CHANNELS)
        wav.setsampwidth(SAMPLE_WIDTH)
        wav.setframerate(WHISPER_RATE)
        wav.writeframes(resampled)
    return output.getvalue()


def multipart_request(wav_data: bytes) -> tuple[bytes, str]:
    """Build the multipart body accepted by whisper.cpp /inference."""
    boundary = "----om6dof-stt-" + uuid.uuid4().hex
    marker = boundary.encode("ascii")
    parts = [
        b"--" + marker + b"\r\n"
        b'Content-Disposition: form-data; name="file"; filename="speech.wav"\r\n'
        b"Content-Type: audio/wav\r\n\r\n" + wav_data + b"\r\n",
        b"--" + marker + b"\r\n"
        b'Content-Disposition: form-data; name="response_format"\r\n\r\n'
        b"json\r\n",
        b"--" + marker + b"\r\n"
        b'Content-Disposition: form-data; name="temperature"\r\n\r\n'
        b"0.0\r\n",
        b"--" + marker + b"--\r\n",
    ]
    return b"".join(parts), boundary


def transcribe(server_url: str, wav_data: bytes, timeout: float) -> str:
    body, boundary = multipart_request(wav_data)
    request = urllib.request.Request(
        server_url,
        data=body,
        headers={"Content-Type": f"multipart/form-data; boundary={boundary}"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        payload = json.loads(response.read().decode("utf-8"))
    text = payload.get("text", "") if isinstance(payload, dict) else ""
    return " ".join(str(text).strip().split())


class UnitreeStt(Node):
    def __init__(self) -> None:
        super().__init__("unitree_stt")
        self.declare_parameter(
            "speech_topic", "/application/audio/speech_s16le")
        self.declare_parameter(
            "voice_active_topic", "/application/audio/voice_active")
        self.declare_parameter("text_topic", "/application/stt/text")
        self.declare_parameter("event_topic", "/application/stt/event")
        self.declare_parameter("status_topic", "/application/stt/status")
        self.declare_parameter(
            "server_url", "http://127.0.0.1:8178/inference")
        self.declare_parameter("min_utterance_ms", 400)
        self.declare_parameter("max_utterance_sec", 20.0)
        self.declare_parameter("continuous_mode", False)
        self.declare_parameter("continuous_chunk_sec", 5.0)
        self.declare_parameter("request_timeout_sec", 60.0)

        speech_topic = str(self.get_parameter("speech_topic").value)
        voice_topic = str(self.get_parameter("voice_active_topic").value)
        text_topic = str(self.get_parameter("text_topic").value)
        event_topic = str(self.get_parameter("event_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        self.server_url = str(self.get_parameter("server_url").value)
        self.min_bytes = max(
            1,
            int(self.get_parameter("min_utterance_ms").value)
            * INPUT_RATE * SAMPLE_WIDTH // 1000,
        )
        self.continuous_mode = bool(
            self.get_parameter("continuous_mode").value)
        chunk_sec = (
            float(self.get_parameter("continuous_chunk_sec").value)
            if self.continuous_mode
            else float(self.get_parameter("max_utterance_sec").value)
        )
        self.max_bytes = max(
            self.min_bytes, int(chunk_sec * INPUT_RATE * SAMPLE_WIDTH))
        self.request_timeout = float(
            self.get_parameter("request_timeout_sec").value)

        stream_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.text_pub = self.create_publisher(String, text_topic, state_qos)
        # Volatile event stream for automatic consumers. Unlike text_topic,
        # this does not replay an old transcript when a consumer restarts.
        self.event_pub = self.create_publisher(String, event_topic, stream_qos)
        self.status_pub = self.create_publisher(String, status_topic, state_qos)
        self.create_subscription(
            UInt8MultiArray, speech_topic, self._on_speech, stream_qos)
        self.create_subscription(Bool, voice_topic, self._on_voice, state_qos)

        self.voice_active = False
        self.buffer = bytearray()
        self.jobs: queue.Queue[bytes | None] = queue.Queue(maxsize=2)
        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()
        self._publish_status("ready")
        self.get_logger().info(
            f"STT: {speech_topic} -> {text_topic} via {self.server_url}; "
            + (
                f"continuous raw-audio chunks ({chunk_sec:.1f} s)"
                if self.continuous_mode else "VAD utterance mode"
            ))

    def _on_voice(self, message: Bool) -> None:
        if self.continuous_mode:
            return
        active = bool(message.data)
        previous = self.voice_active
        self.voice_active = active
        if active and not previous:
            self.buffer.clear()
            self._publish_status("listening")
        elif not active and previous:
            self._submit_buffer()

    def _on_speech(self, message: UInt8MultiArray) -> None:
        data = bytes(message.data)
        if not data:
            return
        if self.continuous_mode:
            if not self.buffer:
                self._publish_status("listening")
            self.buffer.extend(data)
            if len(self.buffer) >= self.max_bytes:
                self._submit_buffer()
            return
        # A speech frame may arrive just before the voice-state callback.
        if not self.voice_active and not self.buffer:
            self._publish_status("listening")
        self.buffer.extend(data)
        if len(self.buffer) >= self.max_bytes:
            self._submit_buffer()

    def _submit_buffer(self) -> None:
        pcm = bytes(self.buffer)
        self.buffer.clear()
        if len(pcm) < self.min_bytes:
            self._publish_status("ready")
            return
        try:
            self.jobs.put_nowait(pcm)
            self._publish_status("transcribing")
        except queue.Full:
            self.get_logger().warning("STT queue full; dropping utterance")
            self._publish_status("busy")

    def _worker(self) -> None:
        while True:
            pcm = self.jobs.get()
            if pcm is None:
                return
            try:
                text = transcribe(
                    self.server_url, pcm_to_wav(pcm), self.request_timeout)
                if text:
                    message = String()
                    message.data = text
                    self.text_pub.publish(message)
                    self.event_pub.publish(message)
                    self.get_logger().info(f"Transcript: {text}")
                self._publish_status(
                    "listening"
                    if self.continuous_mode or self.voice_active else "ready")
            except (OSError, ValueError, json.JSONDecodeError,
                    urllib.error.URLError) as exc:
                self.get_logger().error(f"STT request failed: {exc}")
                self._publish_status("error")
            finally:
                self.jobs.task_done()

    def _publish_status(self, status: str) -> None:
        message = String()
        message.data = status
        self.status_pub.publish(message)

    def destroy_node(self):
        try:
            self.jobs.put_nowait(None)
        except queue.Full:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UnitreeStt()
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
