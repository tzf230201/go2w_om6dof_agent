"""Natural Kokoro speech output for the Unitree built-in speaker.

Each volatile LLM response event is synthesized once, converted to a 44.1 kHz
mono WAV, and uploaded to the Go2W audiohub megaphone service.
``/application/tts/speaking`` implements a half-duplex microphone gate.
"""

from __future__ import annotations

import audioop
import base64
from collections import OrderedDict
import io
import json
import queue
import re
import threading
import time
import wave

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String
from unitree_api.msg import Request, Response


TARGET_RATE = 44100
CHUNK_SIZE = 4096
INTER_CHUNK_DELAY = 0.05
ENTER_MEGAPHONE = 4001
EXIT_MEGAPHONE = 4002
UPLOAD_MEGAPHONE = 4003


def normalize_tts_text(text: str, limit: int = 700) -> str:
    """Collapse whitespace and cap unusually long spoken responses."""
    return " ".join(text.strip().split())[:limit].strip()


def split_tts_segments(text: str, max_chars: int = 48) -> list[str]:
    """Split at natural pauses so the first audio reaches the robot quickly."""
    text = normalize_tts_text(text)
    if not text:
        return []
    segments: list[str] = []
    sentences = re.split(r"(?<=[.!?])\s+", text)
    for sentence in sentences:
        remaining = sentence.strip()
        while len(remaining) > max_chars:
            window = remaining[:max_chars + 1]
            candidates = [
                window.rfind(mark) + 1 for mark in (", ", "; ", ": ", " ")]
            split_at = max(candidates)
            if split_at < max_chars // 2:
                split_at = max_chars
            segment = remaining[:split_at].strip()
            if segment:
                segments.append(segment)
            remaining = remaining[split_at:].strip()
        if remaining:
            segments.append(remaining)
    return segments


def float_audio_to_wav(audio, source_rate: int) -> bytes:
    """Convert Kokoro float audio to Go2W's 44.1 kHz mono WAV format."""
    import numpy as np

    samples = np.asarray(audio, dtype=np.float32).reshape(-1)
    samples = np.clip(samples, -1.0, 1.0)
    pcm = (samples * 32767.0).astype("<i2").tobytes()
    if source_rate != TARGET_RATE:
        pcm, _ = audioop.ratecv(
            pcm, 2, 1, int(source_rate), TARGET_RATE, None)
    output = io.BytesIO()
    with wave.open(output, "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(TARGET_RATE)
        wav.writeframes(pcm)
    return output.getvalue()


class KokoroTts(Node):
    def __init__(self) -> None:
        super().__init__("kokoro_tts")
        self.declare_parameter("event_topic", "/application/llm/event")
        self.declare_parameter("speaking_topic", "/application/tts/speaking")
        self.declare_parameter("status_topic", "/application/tts/status")
        self.declare_parameter("output_topic", "/api/audiohub/request")
        self.declare_parameter(
            "model_path", "/mnt/agx_nvme/kokoro/kokoro-v1.0.onnx")
        self.declare_parameter(
            "voices_path", "/mnt/agx_nvme/kokoro/voices-v1.0.bin")
        self.declare_parameter("voice", "af_heart")
        self.declare_parameter("language", "en-us")
        self.declare_parameter("speed", 1.0)
        self.declare_parameter("tail_silence_ms", 700)
        self.declare_parameter("segment_max_chars", 48)
        self.declare_parameter("stream_segments", False)
        self.declare_parameter("cache_entries", 32)

        event_topic = str(self.get_parameter("event_topic").value)
        speaking_topic = str(self.get_parameter("speaking_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        model_path = str(self.get_parameter("model_path").value)
        voices_path = str(self.get_parameter("voices_path").value)
        self.voice = str(self.get_parameter("voice").value)
        self.language = str(self.get_parameter("language").value)
        self.speed = float(self.get_parameter("speed").value)
        self.tail_silence = max(
            0.0, float(self.get_parameter("tail_silence_ms").value) / 1000.0)
        self.segment_max_chars = max(
            24, int(self.get_parameter("segment_max_chars").value))
        self.stream_segments = bool(
            self.get_parameter("stream_segments").value)
        self.cache_entries = max(
            0, int(self.get_parameter("cache_entries").value))

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
        speaker_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.speaker_pub = self.create_publisher(
            Request, output_topic, speaker_qos)
        self.create_subscription(
            Response, "/api/audiohub/response",
            self._on_audiohub_response, speaker_qos)
        self.speaking_pub = self.create_publisher(
            Bool, speaking_topic, state_qos)
        self.status_pub = self.create_publisher(String, status_topic, state_qos)
        self.create_subscription(String, event_topic, self._on_response, event_qos)

        self._publish_status("loading")
        from kokoro_onnx import Kokoro
        self.model = Kokoro(model_path, voices_path)
        self.wav_cache: OrderedDict[str, tuple[bytes, float]] = OrderedDict()
        self.jobs: queue.Queue[str | None] = queue.Queue(maxsize=3)
        self._closing = False
        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()
        self._publish_speaking(False)
        self._publish_status("ready")
        self.get_logger().info(
            f"Kokoro {self.voice} ready: {event_topic} -> {output_topic}")

    def _on_response(self, message: String) -> None:
        text = normalize_tts_text(message.data)
        if not text:
            return
        try:
            self.jobs.put_nowait(text)
            self._publish_status("queued")
        except queue.Full:
            self.get_logger().warning("TTS queue full; dropping response")

    def _worker(self) -> None:
        while not self._closing:
            text = self.jobs.get()
            if text is None:
                return
            speaking = False
            try:
                started = time.monotonic()
                # Multiple WAV files inside one Go2W megaphone session produce
                # audible gaps on this firmware. Keep the experimental split
                # mode available, but default to one continuous WAV.
                segments = (
                    split_tts_segments(text, self.segment_max_chars)
                    if self.stream_segments else [text]
                )
                self._publish_status("synthesizing")
                first_wav, first_duration, first_cached = self._synthesize(
                    segments[0])
                # Publish the gate first and allow the microphone bridge one
                # scheduling interval to close before speaker playback begins.
                self._publish_speaking(True)
                speaking = True
                time.sleep(0.10)
                self._publish_status("speaking")
                self._audiohub_call(ENTER_MEGAPHONE, {})
                time.sleep(0.10)
                playback_deadline = time.monotonic()
                for index, segment in enumerate(segments):
                    if index == 0:
                        wav_blob = first_wav
                        duration = first_duration
                        cached = first_cached
                    else:
                        wav_blob, duration, cached = self._synthesize(segment)
                    upload_started = time.monotonic()
                    self._upload_wav(wav_blob)
                    uploaded = time.monotonic()
                    # Each completed WAV is queued behind audio that is still
                    # playing. Synthesis of the next segment overlaps playback.
                    playback_deadline = max(
                        playback_deadline, uploaded) + duration
                    if index == 0:
                        self.get_logger().info(
                            f"First TTS segment queued in "
                            f"{uploaded - started:.2f} s "
                            f"(upload {uploaded - upload_started:.2f} s, "
                            f"cache={'hit' if cached else 'miss'})")
                remaining = playback_deadline - time.monotonic()
                time.sleep(max(0.0, remaining) + self.tail_silence)
                self._audiohub_call(EXIT_MEGAPHONE, {})
                time.sleep(0.20)
                self.get_logger().info(f"Spoke: {text}")
            except Exception as exc:
                self.get_logger().error(f"TTS failed: {exc}")
                self._publish_status("error")
            finally:
                if speaking:
                    self._publish_speaking(False)
                if not self._closing:
                    self._publish_status("ready")
                self.jobs.task_done()

    def _synthesize(self, text: str) -> tuple[bytes, float, bool]:
        cached = self.wav_cache.get(text)
        if cached is not None:
            self.wav_cache.move_to_end(text)
            return cached[0], cached[1], True
        audio, sample_rate = self.model.create(
            text, voice=self.voice, speed=self.speed, lang=self.language)
        wav_blob = float_audio_to_wav(audio, sample_rate)
        duration = max(0.0, (len(wav_blob) - 44) / TARGET_RATE / 2)
        if self.cache_entries:
            self.wav_cache[text] = (wav_blob, duration)
            self.wav_cache.move_to_end(text)
            while len(self.wav_cache) > self.cache_entries:
                self.wav_cache.popitem(last=False)
        return wav_blob, duration, False

    def _upload_wav(self, wav_blob: bytes) -> None:
        encoded = base64.b64encode(wav_blob).decode("ascii")
        chunks = [
            encoded[offset:offset + CHUNK_SIZE]
            for offset in range(0, len(encoded), CHUNK_SIZE)
        ]
        for index, chunk in enumerate(chunks, 1):
            self._audiohub_call(UPLOAD_MEGAPHONE, {
                "current_block_size": len(chunk),
                "block_content": chunk,
                "current_block_index": index,
                "total_block_number": len(chunks),
            })
            time.sleep(INTER_CHUNK_DELAY)

    def _publish_speaking(self, speaking: bool) -> None:
        self.speaking_pub.publish(Bool(data=speaking))

    def _audiohub_call(self, api_id: int, parameters: dict) -> None:
        request = Request()
        request.header.identity.id = api_id
        request.header.identity.api_id = api_id
        request.parameter = json.dumps(parameters, ensure_ascii=True)
        self.speaker_pub.publish(request)

    def _on_audiohub_response(self, response: Response) -> None:
        api_id = int(response.header.identity.api_id)
        code = int(response.header.status.code)
        if code != 0:
            self.get_logger().warning(
                f"Audiohub api_id={api_id} returned code={code}: "
                f"{response.data[:160]}")

    def _publish_status(self, status: str) -> None:
        self.status_pub.publish(String(data=status))

    def destroy_node(self):
        self._closing = True
        self._publish_speaking(False)
        try:
            self.jobs.put_nowait(None)
        except queue.Full:
            pass
        if self.worker.is_alive():
            self.worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KokoroTts()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
