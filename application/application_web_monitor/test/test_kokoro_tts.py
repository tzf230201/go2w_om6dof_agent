import wave
from io import BytesIO

import numpy as np

from application_web_monitor.kokoro_tts import (
    TARGET_RATE,
    float_audio_to_wav,
    normalize_tts_text,
    split_tts_segments,
)


def test_normalize_tts_text_collapses_whitespace_and_limits_length():
    assert normalize_tts_text("  Hello\n  robot.  ") == "Hello robot."
    assert normalize_tts_text("abcdef", limit=4) == "abcd"


def test_float_audio_to_wav_uses_unitree_speaker_format():
    blob = float_audio_to_wav(np.zeros(2400, dtype=np.float32), 24000)
    with wave.open(BytesIO(blob), "rb") as wav:
        assert wav.getnchannels() == 1
        assert wav.getsampwidth() == 2
        assert wav.getframerate() == TARGET_RATE
        assert wav.getnframes() > 4000


def test_split_tts_segments_prefers_sentences_and_bounds_long_text():
    text = "Hello. This is a deliberately longer sentence for the robot voice."
    segments = split_tts_segments(text, max_chars=32)
    assert segments[0] == "Hello."
    assert " ".join(segments) == text
    assert all(len(segment) <= 32 for segment in segments)
