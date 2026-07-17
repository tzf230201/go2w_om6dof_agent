"""Decode the Go2W built-in microphone Opus stream on the AGX.

The robot publishes one 20 ms Opus packet on ``/audiosender`` at 50 Hz. This
node decodes it to mono 48 kHz signed 16-bit little-endian PCM. SpeexDSP then
suppresses stationary noise and gates non-speech frames. The filtered PCM is
shared by the web monitor and speech-to-text consumers.
"""

import ctypes

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String, UInt8MultiArray
from unitree_go.msg import AudioData


SAMPLE_RATE = 48000
CHANNELS = 1
MAX_FRAME_SAMPLES = 5760
FRAME_SAMPLES = 960
FRAME_DURATION_MS = 20

SPEEX_PREPROCESS_SET_DENOISE = 0
SPEEX_PREPROCESS_SET_VAD = 4
SPEEX_PREPROCESS_SET_DEREVERB = 8
SPEEX_PREPROCESS_SET_PROB_START = 14
SPEEX_PREPROCESS_SET_PROB_CONTINUE = 16
SPEEX_PREPROCESS_SET_NOISE_SUPPRESS = 18


class OpusDecoder:
    def __init__(self) -> None:
        self.lib = ctypes.CDLL("libopus.so.0")
        self.lib.opus_decoder_create.argtypes = [
            ctypes.c_int32,
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_int),
        ]
        self.lib.opus_decoder_create.restype = ctypes.c_void_p
        self.lib.opus_decode.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_ubyte),
            ctypes.c_int32,
            ctypes.POINTER(ctypes.c_int16),
            ctypes.c_int,
            ctypes.c_int,
        ]
        self.lib.opus_decode.restype = ctypes.c_int
        self.lib.opus_decoder_destroy.argtypes = [ctypes.c_void_p]

        error = ctypes.c_int()
        self.handle = self.lib.opus_decoder_create(
            SAMPLE_RATE, CHANNELS, ctypes.byref(error))
        if not self.handle or error.value != 0:
            raise RuntimeError(f"opus_decoder_create failed: {error.value}")
        self.output = (ctypes.c_int16 * MAX_FRAME_SAMPLES)()

    def decode(self, packet: bytes) -> bytes:
        encoded = (ctypes.c_ubyte * len(packet)).from_buffer_copy(packet)
        samples = self.lib.opus_decode(
            self.handle,
            encoded,
            len(packet),
            self.output,
            MAX_FRAME_SAMPLES,
            0,
        )
        if samples < 0:
            raise RuntimeError(f"opus_decode failed: {samples}")
        return ctypes.string_at(ctypes.addressof(self.output), samples * 2)

    def close(self) -> None:
        if self.handle:
            self.lib.opus_decoder_destroy(self.handle)
            self.handle = None


class SpeexVoiceFilter:
    """In-place stationary-noise suppression and speech detection."""

    def __init__(
            self,
            noise_suppress_db: int,
            speech_probability: int,
            continue_probability: int) -> None:
        self.lib = ctypes.CDLL("libspeexdsp.so.1")
        self.lib.speex_preprocess_state_init.argtypes = [
            ctypes.c_int,
            ctypes.c_int,
        ]
        self.lib.speex_preprocess_state_init.restype = ctypes.c_void_p
        self.lib.speex_preprocess_ctl.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
        ]
        self.lib.speex_preprocess_ctl.restype = ctypes.c_int
        self.lib.speex_preprocess_run.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_int16),
        ]
        self.lib.speex_preprocess_run.restype = ctypes.c_int
        self.lib.speex_preprocess_state_destroy.argtypes = [ctypes.c_void_p]

        self.handle = self.lib.speex_preprocess_state_init(
            FRAME_SAMPLES, SAMPLE_RATE)
        if not self.handle:
            raise RuntimeError("speex_preprocess_state_init failed")
        self._set(SPEEX_PREPROCESS_SET_DENOISE, 1)
        self._set(SPEEX_PREPROCESS_SET_VAD, 1)
        self._set(SPEEX_PREPROCESS_SET_DEREVERB, 1)
        self._set(SPEEX_PREPROCESS_SET_NOISE_SUPPRESS, noise_suppress_db)
        self._set(SPEEX_PREPROCESS_SET_PROB_START, speech_probability)
        self._set(
            SPEEX_PREPROCESS_SET_PROB_CONTINUE, continue_probability)

    def _set(self, request: int, value: int) -> None:
        setting = ctypes.c_int(value)
        result = self.lib.speex_preprocess_ctl(
            self.handle, request, ctypes.byref(setting))
        if result != 0:
            raise RuntimeError(
                f"speex_preprocess_ctl({request}) failed: {result}")

    def process(self, pcm: bytes):
        if len(pcm) != FRAME_SAMPLES * 2:
            raise RuntimeError(
                f"unexpected PCM frame size: {len(pcm)} bytes")
        samples = (ctypes.c_int16 * FRAME_SAMPLES).from_buffer_copy(pcm)
        speech = bool(self.lib.speex_preprocess_run(self.handle, samples))
        filtered = ctypes.string_at(
            ctypes.addressof(samples), FRAME_SAMPLES * 2)
        return filtered, speech

    def close(self) -> None:
        if self.handle:
            self.lib.speex_preprocess_state_destroy(self.handle)
            self.handle = None


class UnitreeAudioBridge(Node):
    def __init__(self) -> None:
        super().__init__("unitree_audio_bridge")
        self.declare_parameter("input_topic", "/audiosender")
        self.declare_parameter(
            "pcm_topic", "/application/audio/pcm_s16le")
        self.declare_parameter(
            "speech_topic", "/application/audio/speech_s16le")
        self.declare_parameter(
            "voice_active_topic", "/application/audio/voice_active")
        self.declare_parameter("format_topic", "/application/audio/format")
        self.declare_parameter("noise_suppress_db", -24)
        self.declare_parameter("speech_probability", 65)
        self.declare_parameter("continue_probability", 45)
        self.declare_parameter("hangover_ms", 500)
        self.declare_parameter("audio_filter_enabled", False)

        input_topic = str(self.get_parameter("input_topic").value)
        pcm_topic = str(self.get_parameter("pcm_topic").value)
        speech_topic = str(self.get_parameter("speech_topic").value)
        voice_active_topic = str(
            self.get_parameter("voice_active_topic").value)
        format_topic = str(self.get_parameter("format_topic").value)
        noise_suppress_db = int(
            self.get_parameter("noise_suppress_db").value)
        speech_probability = int(
            self.get_parameter("speech_probability").value)
        continue_probability = int(
            self.get_parameter("continue_probability").value)
        hangover_ms = max(0, int(self.get_parameter("hangover_ms").value))
        self.audio_filter_enabled = bool(
            self.get_parameter("audio_filter_enabled").value)
        self.decoder = OpusDecoder()
        self.voice_filter = SpeexVoiceFilter(
            noise_suppress_db,
            speech_probability,
            continue_probability,
        )
        self.frames = 0
        self.hangover_frames = round(hangover_ms / FRAME_DURATION_MS)
        self.hangover_remaining = 0
        self.voice_active = False
        self.silence = bytes(FRAME_SAMPLES * 2)

        stream_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        format_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pcm_pub = self.create_publisher(
            UInt8MultiArray, pcm_topic, stream_qos)
        self.speech_pub = self.create_publisher(
            UInt8MultiArray, speech_topic, stream_qos)
        self.format_pub = self.create_publisher(
            String, format_topic, format_qos)
        self.voice_pub = self.create_publisher(
            Bool, voice_active_topic, format_qos)
        self.audio_sub = self.create_subscription(
            AudioData, input_topic, self._on_audio, stream_qos)

        format_message = String()
        processing = "speexdsp" if self.audio_filter_enabled else "none"
        format_message.data = (
            '{"encoding":"pcm_s16le","sample_rate":48000,"channels":1,'
            f'"noise_suppression":"{processing}","vad":true,'
            f'"continuous_stt":{str(not self.audio_filter_enabled).lower()}}}'
        )
        self.format_pub.publish(format_message)
        self._publish_voice_state(False, force=True)
        self.get_logger().info(
            f"Unitree microphone: {input_topic} (Opus) -> {pcm_topic} "
            f"(filtered PCM); speech-only -> {speech_topic}; "
            f"audio filter {'enabled' if self.audio_filter_enabled else 'BYPASSED'}, "
            f"noise suppression {noise_suppress_db} dB, VAD "
            f"{speech_probability}/{continue_probability}, "
            f"hangover {hangover_ms} ms")

    def _on_audio(self, message: AudioData) -> None:
        packet = bytes(message.data)
        if not packet:
            return
        try:
            pcm = self.decoder.decode(packet)
            filtered, detected = self.voice_filter.process(pcm)
        except RuntimeError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=5.0)
            return

        if detected:
            self.hangover_remaining = self.hangover_frames
        elif self.hangover_remaining > 0:
            self.hangover_remaining -= 1
        voice_active = detected or self.hangover_remaining > 0
        self._publish_voice_state(voice_active)

        forwarded = filtered if self.audio_filter_enabled else pcm
        output = UInt8MultiArray()
        output.data = (
            forwarded
            if voice_active or not self.audio_filter_enabled
            else self.silence
        )
        self.pcm_pub.publish(output)
        if voice_active or not self.audio_filter_enabled:
            speech_output = UInt8MultiArray()
            speech_output.data = forwarded
            self.speech_pub.publish(speech_output)
        self.frames += 1
        if self.frames == 1:
            self.get_logger().info(
                f"First microphone frame decoded ({len(packet)} Opus bytes -> "
                f"{len(pcm)} PCM bytes)")

    def _publish_voice_state(self, active: bool, force: bool = False) -> None:
        if not force and active == self.voice_active:
            return
        self.voice_active = active
        message = Bool()
        message.data = active
        self.voice_pub.publish(message)
        self.get_logger().info(
            "Human voice detected" if active else "Noise-only / silence")

    def destroy_node(self):
        self.voice_filter.close()
        self.decoder.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UnitreeAudioBridge()
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
