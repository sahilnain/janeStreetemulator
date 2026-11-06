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


def i2c_trace(seq, h=20, scl=6, sda=7, stuck=False):
    """seq: list of bits with S/P markers. SCL 1 and SDA 1 when idle."""
    s = bytearray()
    lc, ld = 1, 1

    def put(n):
        s.extend([(lc << scl | ld << sda)] * n)

    put(20)
    for x in seq:
        if x == "S":
            ld = 0
            put(h)
            lc = 0
            put(h)
        elif x == "P":
            ld = 0
            put(h)
            lc = 1
            put(h)
            ld = 1
            put(h)
        else:
            ld = x
            put(h)
            lc = 1
            put(h)
            lc = 0
    put(20)
    return s


def bits(b):
    return [(b >> (7 - k)) & 1 for k in range(8)]


def test_i2c_good_write_decoded():
    tr = i2c_trace(["S"] + bits(0xA0) + [0] + bits(0x12) + [0] + ["P"])
    assert sigrok.i2c(tr, 6, 7)["ev"] == ["S", ("AW", 0x50), "A", ("DW", 0x12), "A", "P"]


def test_i2c_nack_seen():
    tr = i2c_trace(["S"] + bits(0xA1) + [1] + ["P"])
    assert sigrok.i2c(tr, 6, 7)["ev"] == ["S", ("AR", 0x50), "N", "P"]


def test_i2c_timing_flags_fast_and_broken():
    import i2c_lib as L
    tr = i2c_trace(["S"] + bits(0xA0) + [0] + ["P"], h=10)
    assert L.timing(tr, "SM") and L.timing(tr, "FM")     # 10 cycles = 208 ns: far too fast for both
    assert not L.timing(i2c_trace(["S"] + bits(0xA0) + [0] + ["P"], h=300), "SM", check=("low", "high", "fscl"))


def usb_trace(levels, speed="FS", bit=None):
    import usb_lib as U
    sp = U.SPEEDS[speed]
    bit = bit or sp["B"]
    idle = 1 << sp["jp"]
    code = {U.J: 1 << sp["jp"], U.K: 1 << sp["kp"], U.SE0: 0}
    s = bytearray([idle] * 100)
    for lv in levels:
        s += bytes([code[lv]]) * bit
    s += bytes([idle]) * (20 * bit)
    return s


def test_usb_crc_matches_the_catalogue():
    """CRC-5/USB and CRC-16/USB check values from the CRC catalogue, CRC-16 also via crcmod."""
    import crcmod.predefined as p
    import usb_lib as U
    msg = b"123456789"
    bits = [(c >> i) & 1 for c in msg for i in range(8)]
    c5 = U.crc5(bits)
    assert sum(b << i for i, b in enumerate(c5)) == 0x19
    c16 = U.crc16_bits(list(msg))
    assert sum(b << i for i, b in enumerate(c16)) == 0xB4C8 == p.mkCrcFun("crc-16-usb")(msg)


def test_usb_good_packets_clean():
    import usb_lib as U
    for pkt, name in ((U.token(U.IN, 0x2A, 3), "IN"), (U.data_packet(U.DATA1, [1, 2, 3]), "DATA1"), ([U.ACK], "ACK")):
        for speed in ("FS", "LS"):
            d = sigrok.usb(usb_trace(U.encode(pkt), speed), 0, 1, speed)
            assert not d["errors"], d["errors"]
            assert [p[1] for p in d["packets"]] == [name]


def test_usb_decoder_flags_bad_packets():
    import usb_lib as U
    a = sigrok.usb(usb_trace(U.encode(U.token(U.IN, 0x2A, 3, bad_crc=True))), 0, 1, "FS")
    assert any(e[1] == "crc5-err" for e in a["errors"])
    a = sigrok.usb(usb_trace(U.encode(U.data_packet(U.DATA1, [1, 2, 3], bad_crc=True))), 0, 1, "FS")
    assert any(e[1] == "crc16-err" for e in a["errors"])
    a = sigrok.usb(usb_trace(U.encode(U.data_packet(U.DATA1, [0xFF, 0xFF, 0x12]), no_stuff_first=True)), 0, 1, "FS")
    assert any("stuff" in e[2].lower() for e in a["errors"])
    a = sigrok.usb(usb_trace(U.encode(U.token(U.IN, 0x2A, 3), sync_bits=6)), 0, 1, "FS")
    assert any(e[1] == "sync-err" for e in a["errors"])
