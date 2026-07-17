import io
import struct
import wave

from application_web_monitor.unitree_stt import multipart_request, pcm_to_wav


def test_pcm_to_wav_resamples_to_16khz():
    pcm = struct.pack("<" + "h" * 4800, *([100] * 4800))
    result = pcm_to_wav(pcm)
    with wave.open(io.BytesIO(result), "rb") as wav:
        assert wav.getframerate() == 16000
        assert wav.getnchannels() == 1
        assert wav.getsampwidth() == 2
        assert 1590 <= wav.getnframes() <= 1610


def test_multipart_contains_wav_and_options():
    body, boundary = multipart_request(b"RIFF-test")
    assert boundary.encode("ascii") in body
    assert b'filename="speech.wav"' in body
    assert b"RIFF-test" in body
    assert b'response_format' in body
    assert body.endswith(("--" + boundary + "--\r\n").encode("ascii"))
