# SPDX-License-Identifier: Apache-2.0
"""
test_controls: negative controls. The decoders must flag traces that are wrong on purpose:
if they could not, a pass in the other tests would prove nothing. Traces are built here by hand.
"""

import pe_fw as F
import sigrok
from uart_lib import frame


def trace(pin, seg, B, idle=1):
    out = bytearray([idle << pin] * 50)
    for lv, n in seg:
        out += bytes([lv << pin]) * int(n * B)
    out += bytes([idle << pin]) * (20 * B)
    return out


def test_uart_good_is_clean():
    d = sigrok.uart(trace(0, frame(0xA5), 104), 0, 48_000_000 // 104)
    assert d["data"] == [0xA5] and not d["warn"]


def test_uart_bad_stop_flagged():
    assert sigrok.uart(trace(0, frame(0xA5, bad_stop=True), 104), 0, 48_000_000 // 104)["warn"]


def test_uart_bad_parity_flagged():
    d = sigrok.uart(trace(0, frame(0xA5, 8, F.PAR_EVEN, bad_parity=True), 104), 0, 48_000_000 // 104, parity="even")
    assert d["perr"]


def test_uart_wrong_baud_flagged():
    d = sigrok.uart(trace(0, frame(0xA5) * 4, 104), 0, 48_000_000 // 80)
    assert d["warn"] or d["data"] != [0xA5] * 4


def spi_trace(data, cpol=0, cpha=0, h=8, sck=2, mosi=3, cs=5, flip_cpha=False):
    s = bytearray()
    lv = {sck: cpol, mosi: 0, cs: 1}

    def put(n):
        s.extend([sum(v << p for p, v in lv.items()) | 0x10] * n)

    put(20)
    lv[cs] = 0
    put(20)
    for d in data:
        for k in range(8):
            lv[mosi] = (d >> (7 - k)) & 1
            put(h)
            lv[sck] = 1 - cpol
            put(h)
            lv[sck] = cpol
    put(20)
    lv[cs] = 1
    put(20)
    return s


def test_spi_good_is_clean():
    assert sigrok.spi(spi_trace([0xA5, 0x3C]), 2, 3, None, 5)["mosi"] == [0xA5, 0x3C]


def test_spi_wrong_phase_reads_wrong_data():
    assert sigrok.spi(spi_trace([0xA5, 0x3C]), 2, 3, None, 5, cpha=1)["mosi"] != [0xA5, 0x3C]


def test_spi_check_frames_flags_bad_traces():
    import pytest
    from spi_lib import check_frames
    with pytest.raises(AssertionError):
        check_frames(spi_trace([0xA5]), 0, 1)             # data changes on the sampling edge for CPHA 1
    good = spi_trace([0xA5])
    assert check_frames(good, 0, 0, 16) == 1
    with pytest.raises(AssertionError):
        check_frames(good, 0, 0, 14)                      # wrong number of edges
    with pytest.raises(AssertionError):
        check_frames(good, 1, 0)                          # SCK not at the idle level
