"""Capture the DJI Wireless Mic receiver on the AGX and publish ROS PCM.

The USB receiver exposes two identical 48 kHz channels. Capture goes through
the PulseAudio source so this service can coexist with the desktop audio server
that owns the ALSA device after boot. The node mixes the channels to mono,
applies SpeexDSP noise suppression, and publishes the application audio topics.
"""

from __future__ import annotations

import audioop
from collections import deque
import subprocess
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String, UInt8MultiArray

from application_web_monitor.unitree_audio_bridge import SpeexVoiceFilter


SAMPLE_RATE = 48000
FRAME_SAMPLES = 960
SAMPLE_WIDTH = 2
INPUT_CHANNELS = 2
INPUT_FRAME_BYTES = FRAME_SAMPLES * SAMPLE_WIDTH * INPUT_CHANNELS


def stereo_to_mono(data: bytes) -> bytes:
    """Mix stereo S16LE equally without attenuating identical channels."""
    return audioop.tomono(data, SAMPLE_WIDTH, 0.5, 0.5)


def read_exact(stream, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        part = stream.read(size - len(chunks))
        if not part:
            return b""
        chunks.extend(part)
    return bytes(chunks)


def select_pulse_source(pactl_output: str, requested: str = "") -> str:
    if requested.strip():
        return requested.strip()
    for line in pactl_output.splitlines():
        fields = line.split("\t")
        if len(fields) >= 2 and "DJI" in fields[1] and ".monitor" not in fields[1]:
            return fields[1]
    raise RuntimeError("DJI Wireless Mic PulseAudio source was not found")


def find_dji_pulse_source(requested: str = "") -> str:
    if requested.strip():
        return requested.strip()
    result = subprocess.run(
        ["pactl", "list", "short", "sources"],
        capture_output=True,
        text=True,
        timeout=4.0,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"cannot query PulseAudio sources: {result.stderr.strip()}")
    return select_pulse_source(result.stdout)


def unmute_pulse_source(source: str) -> None:
    result = subprocess.run(
        ["pactl", "set-source-mute", source, "0"],
        capture_output=True,
        text=True,
        timeout=4.0,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"cannot unmute DJI PulseAudio source: {result.stderr.strip()}")


class DjiAudioBridge(Node):
    def __init__(self) -> None:
        super().__init__("dji_audio_bridge")
        self.declare_parameter("pulse_source", "")
        self.declare_parameter("pcm_topic", "/application/audio/pcm_s16le")
        self.declare_parameter(
            "speech_topic", "/application/audio/speech_s16le")
        self.declare_parameter(
            "voice_active_topic", "/application/audio/voice_active")
        self.declare_parameter("format_topic", "/application/audio/format")
        self.declare_parameter("level_threshold", 400)
        self.declare_parameter("hangover_ms", 1200)
        self.declare_parameter("pre_roll_ms", 300)
        self.declare_parameter("noise_suppress_db", -18)
        self.declare_parameter("tts_speaking_topic", "/application/tts/speaking")
        self.declare_parameter("tts_resume_delay_ms", 700)

        source = find_dji_pulse_source(
            str(self.get_parameter("pulse_source").value))
        unmute_pulse_source(source)
        pcm_topic = str(self.get_parameter("pcm_topic").value)
        speech_topic = str(self.get_parameter("speech_topic").value)
        voice_topic = str(self.get_parameter("voice_active_topic").value)
        format_topic = str(self.get_parameter("format_topic").value)
        self.level_threshold = max(
            0, int(self.get_parameter("level_threshold").value))
        self.hangover_frames = max(
            0, int(self.get_parameter("hangover_ms").value) // 20)
        pre_roll_frames = max(
            1, int(self.get_parameter("pre_roll_ms").value) // 20)
        noise_suppress_db = int(
            self.get_parameter("noise_suppress_db").value)
        tts_speaking_topic = str(
            self.get_parameter("tts_speaking_topic").value)
        self.tts_resume_delay = max(
            0.0, float(self.get_parameter("tts_resume_delay_ms").value) / 1000.0)
        self.voice_filter = SpeexVoiceFilter(
            noise_suppress_db,
            speech_probability=55,
            continue_probability=35,
        )
        self.pre_roll = deque(maxlen=pre_roll_frames)
        self.hangover_remaining = 0
        self.voice_active = False
        self.frames = 0
        self._closing = False
        self.tts_speaking = False
        self.audio_resume_at = 0.0
        self.silence = bytes(FRAME_SAMPLES * SAMPLE_WIDTH)

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
        self.pcm_pub = self.create_publisher(
            UInt8MultiArray, pcm_topic, stream_qos)
        self.speech_pub = self.create_publisher(
            UInt8MultiArray, speech_topic, stream_qos)
        self.voice_pub = self.create_publisher(Bool, voice_topic, state_qos)
        self.format_pub = self.create_publisher(String, format_topic, state_qos)
        self.create_subscription(
            Bool, tts_speaking_topic, self._on_tts_speaking, state_qos)

        self.process = subprocess.Popen(
            [
                "parec", "--record", f"--device={source}",
                "--format=s16le", f"--rate={SAMPLE_RATE}",
                f"--channels={INPUT_CHANNELS}",
                "--latency-msec=20", "--process-time-msec=20",
                "--client-name=application-dji-audio-bridge", "--raw",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        self._publish_voice(False, force=True)
        format_message = String()
        format_message.data = (
            '{"encoding":"pcm_s16le","sample_rate":48000,"channels":1,'
            '"source":"dji_wireless_mic_rx",'
            f'"noise_suppression":"speexdsp_{noise_suppress_db}db",'
            '"continuous_stt":false}'
        )
        self.format_pub.publish(format_message)
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()
        self.get_logger().info(
            f"DJI microphone: PulseAudio {source} -> {pcm_topic} and "
            f"{speech_topic}; "
            f"SpeexDSP noise suppression {noise_suppress_db} dB")

    def _read_loop(self) -> None:
        assert self.process.stdout is not None
        while not self._closing:
            stereo = read_exact(self.process.stdout, INPUT_FRAME_BYTES)
            if not stereo:
                break
            mono = stereo_to_mono(stereo)
            if self.tts_speaking or time.monotonic() < self.audio_resume_at:
                self.hangover_remaining = 0
                self.pre_roll.clear()
                self._publish_voice(False)
                # Keep the web PCM stream alive, but publish digital silence
                # and never feed robot playback back into STT.
                self.pcm_pub.publish(UInt8MultiArray(data=self.silence))
                continue
            filtered, _ = self.voice_filter.process(mono)
            level = audioop.rms(filtered, SAMPLE_WIDTH)
            was_voice_active = self.voice_active
            detected = level >= self.level_threshold
            if detected:
                self.hangover_remaining = self.hangover_frames
            elif self.hangover_remaining > 0:
                self.hangover_remaining -= 1
            voice_active = detected or self.hangover_remaining > 0
            self._publish_voice(voice_active)

            message = UInt8MultiArray()
            message.data = filtered
            self.pcm_pub.publish(message)
            if voice_active:
                if not was_voice_active:
                    for earlier_frame in self.pre_roll:
                        earlier = UInt8MultiArray()
                        earlier.data = earlier_frame
                        self.speech_pub.publish(earlier)
                speech = UInt8MultiArray()
                speech.data = filtered
                self.speech_pub.publish(speech)
            self.pre_roll.append(filtered)
            self.frames += 1
            if self.frames == 1:
                self.get_logger().info(
                    f"First DJI frame captured ({len(mono)} mono PCM bytes)")
        if not self._closing:
            error = ""
            if self.process.stderr is not None:
                error = self.process.stderr.read().decode(
                    "utf-8", "replace").strip()
            self.get_logger().error(
                f"DJI capture stopped (exit {self.process.poll()}): {error}")
            # Do not leave a healthy-looking ROS process with no audio. Ending
            # the node lets systemd Restart=always retry after USB/PulseAudio is
            # ready or after the receiver is reconnected.
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

    def _on_tts_speaking(self, message: Bool) -> None:
        if message.data:
            if not self.tts_speaking:
                self.get_logger().info(
                    "Robot is speaking; microphone -> STT is paused")
            self.tts_speaking = True
            self.audio_resume_at = 0.0
            self.hangover_remaining = 0
            self.pre_roll.clear()
            self._publish_voice(False)
        else:
            was_speaking = self.tts_speaking
            self.tts_speaking = False
            self.audio_resume_at = time.monotonic() + self.tts_resume_delay
            if was_speaking:
                self.get_logger().info(
                    f"Robot finished speaking; microphone resumes in "
                    f"{self.tts_resume_delay:.2f} s")

    def _publish_voice(self, active: bool, force: bool = False) -> None:
        if not force and active == self.voice_active:
            return
        self.voice_active = active
        self.voice_pub.publish(Bool(data=active))

    def destroy_node(self):
        self._closing = True
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
        if self.reader.is_alive():
            self.reader.join(timeout=2.0)
        self.voice_filter.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DjiAudioBridge()
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
