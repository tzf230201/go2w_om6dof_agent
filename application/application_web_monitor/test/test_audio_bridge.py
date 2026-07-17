from application_web_monitor.unitree_audio_bridge import (
    FRAME_SAMPLES,
    SpeexVoiceFilter,
)


def test_speex_filter_keeps_frame_shape_and_rejects_silence():
    voice_filter = SpeexVoiceFilter(-30, 85, 70)
    try:
        filtered = b""
        speech = True
        for _ in range(10):
            filtered, speech = voice_filter.process(bytes(FRAME_SAMPLES * 2))
        assert len(filtered) == FRAME_SAMPLES * 2
        assert speech is False
    finally:
        voice_filter.close()


def test_raw_mode_decision_can_forward_original_pcm():
    """The bridge's raw mode must not substitute Speex-processed samples."""
    pcm = (b"\x34\x12\xcc\xed" * (FRAME_SAMPLES // 2))
    voice_filter = SpeexVoiceFilter(-24, 65, 45)
    try:
        filtered, _ = voice_filter.process(pcm)
        forwarded = pcm
        assert len(filtered) == len(pcm)
        assert forwarded == pcm
    finally:
        voice_filter.close()
