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
