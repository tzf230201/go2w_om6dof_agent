import io
import struct

import pytest

from application_web_monitor.dji_audio_bridge import (
    read_exact,
    select_pulse_source,
    stereo_to_mono,
)


def test_stereo_to_mono_preserves_identical_channel_level():
    stereo = b"".join(struct.pack("<hh", value, value)
                      for value in (1000, -2000, 3000, -4000))
    mono = stereo_to_mono(stereo)
    assert struct.unpack("<hhhh", mono) == (1000, -2000, 3000, -4000)


def test_read_exact_combines_short_reads():
    assert read_exact(io.BytesIO(b"abcdef"), 6) == b"abcdef"
    assert read_exact(io.BytesIO(b"abc"), 6) == b""


def test_select_pulse_source_finds_dji_and_honors_override():
    sources = (
        "0\talsa_input.platform-sound.analog-stereo\tmodule\n"
        "3\talsa_input.usb-DJI_Wireless_Mic_Rx.analog-stereo\tmodule\n"
    )
    assert select_pulse_source(sources) \
        == "alsa_input.usb-DJI_Wireless_Mic_Rx.analog-stereo"
    assert select_pulse_source(sources, "custom.source") == "custom.source"
    with pytest.raises(RuntimeError, match="was not found"):
        select_pulse_source("0\talsa_input.platform\tmodule\n")
