# SPDX-License-Identifier: Apache-2.0
"""
test_uart: the chip model against the UART conventions (docs/protocol_conformance.md Section 2),
judged by the sigrok uart decoder.

TX: the chip transmits and the decoder reads the trace. RX: a stimulus built from the frame
definition drives the chip, the decoder confirms the stimulus is what it should be, and the host
reads the bytes back. Firmware: pe_fw.uart_ex.
"""

import itertools
import random

import pe_fw as F
import pytest
import sigrok
from bench import A_STATUS, CLOCK_HZ, OWN_TX, Bench, Script, owners
from pe_model.blocks import A_IRQ_EN
from uart_lib import boot, check_grid, drain, frame, fw, play, rx_words, transfer, tx_words

PARITY = {F.PAR_NONE: "none", F.PAR_EVEN: "even", F.PAR_ODD: "odd", F.PAR_MARK: "one", F.PAR_SPACE: "zero"}
STOP = {2: 1.0, 3: 1.5, 4: 2.0}
FRAMES = list(itertools.product((5, 6, 7, 8, 9), PARITY, (2, 3, 4), (0, 1), (0, 1)))
LET = {F.PAR_NONE: 'n', F.PAR_EVEN: 'e', F.PAR_ODD: 'o', F.PAR_MARK: 'm', F.PAR_SPACE: 's'}
FRAME_IDS = [f"{b}{LET[p]}{s}{'i' if i else ''}{'r' if m else ''}" for b, p, s, i, m in FRAMES]


def words_for(bits, seed):
    r = random.Random(seed)
    m = (1 << bits) - 1
    return [0, m, 0x155 & m, 0x2AA & m, 1, 1 << (bits - 1)] + [r.randint(0, m) for _ in range(6)]


def decode(samples, pin, B, bits, parity, stop_b2, inv=0, msb=0):
    # Stop bits beyond the first are not read by a receiver (16550 Section 8.1), so 2 is asked as 1.
    return sigrok.uart(samples, pin, CLOCK_HZ // B, bits, PARITY[parity], min(STOP[stop_b2], 1.5), bool(msb), bool(inv))


@pytest.mark.parametrize("bits,parity,stop,inv,msb", FRAMES, ids=FRAME_IDS)
def test_tx_every_frame_format(bits, parity, stop, inv, msb):
    B = 104
    b, nd = boot(B, bits=bits, parity=parity, stop_b2=stop, inv=inv, msb=msb)
    start = len(b.samples)
    words = words_for(bits, bits * 100 + parity * 10 + stop)
    transfer(b, nd, tx_words(words, bits))
    drain(b, nd, B, bits + 5)
    d = decode(b.samples[start:], 0, B, bits, parity, stop, inv, msb)
    assert d["data"] == words and not d["warn"] and not d["perr"]
    check_grid(b.samples[start:], 0, B, bits + (parity != F.PAR_NONE), inv)


@pytest.mark.parametrize("bits,parity,stop,inv,msb", FRAMES, ids=FRAME_IDS)
def test_rx_every_frame_format(bits, parity, stop, inv, msb):
    B = 104
    b, nd = boot(B, bits=bits, parity=parity, stop_b2=stop, inv=inv, msb=msb)
    words = words_for(bits, bits * 100 + parity * 10 + stop + 1)
    seg = [s for w in words for s in frame(w, bits, parity, STOP[stop], inv, msb)]
    start = len(b.samples)
    b.devices.append(Script(play(1, seg, B, idle=0 if inv else 1, lead=20)))
    got = transfer(b, nd, n_rx=len(words) * (2 if bits > 8 else 1), limit=len(seg) * B + 20000)
    b.run(4 * B)
    d = decode(b.samples[start:], 1, B, bits, parity, stop, inv, msb)
    assert d["data"] == words and not d["warn"] and not d["perr"], "the stimulus itself is wrong"
    assert rx_words(got, bits) == words
    assert nd.peek(1)[0] in (None, 0), "parity errors counted on good frames"


BAUDS = [(9600, 5000), (19200, 2500), (38400, 1250), (57600, 833), (115200, 417), (230400, 208), (460800, 104),
         (921600, 52), (1000000, 48), (1500000, 32), (2000000, 24), (3000000, 16)]


def count_for(B):
    return 3 if B >= 2500 else 6 if B >= 800 else 16


@pytest.mark.parametrize("baud,B", BAUDS, ids=[str(x[0]) for x in BAUDS])
def test_tx_standard_bauds(baud, B):
    assert abs(CLOCK_HZ / B / baud - 1) < 0.002, "baud error above 0.2%"
    b, nd = boot(B)
    words = words_for(8, B)[:count_for(B)]
    start = len(b.samples)
    transfer(b, nd, words, limit=len(words) * 12 * B + 40000)
    drain(b, nd, B)
    d = sigrok.uart(b.samples[start:], 0, baud)
    assert d["data"] == words and not d["warn"]
    check_grid(b.samples[start:], 0, B, 8)


@pytest.mark.parametrize("baud,B", BAUDS, ids=[str(x[0]) for x in BAUDS])
def test_rx_standard_bauds(baud, B):
    b, nd = boot(B)
    words = words_for(8, B + 1)[:count_for(B)]
    bit = CLOCK_HZ / baud                       # the sender uses the exact baud, not B
    seg = [s for w in words for s in frame(w)]
    start = len(b.samples)
    b.devices.append(Script(play(1, seg, bit, lead=20)))
    got = transfer(b, nd, n_rx=len(words), limit=len(seg) * B + 40000)
    assert got == words
    assert sigrok.uart(b.samples[start:], 1, baud)["data"] == words


@pytest.mark.parametrize("B", (52, 104, 417))
def test_tx_all_byte_values(B):
    b, nd = boot(B)
    data = list(range(256))
    start = len(b.samples)
    transfer(b, nd, data, limit=280 * 12 * B)
    drain(b, nd, B)
    d = sigrok.uart(b.samples[start:], 0, CLOCK_HZ // B)
    assert d["data"] == data and not d["warn"]


@pytest.mark.parametrize("B", (52, 104, 417))
def test_rx_all_byte_values_back_to_back(B):
    b, nd = boot(B)
    data = list(range(256))
    seg = [s for w in data for s in frame(w)]
    b.devices.append(Script(play(1, seg, B, lead=20)))
    assert transfer(b, nd, n_rx=256, limit=280 * 12 * B) == data


TOL = (-3.0, -2.0, -1.0, 1.0, 2.0, 3.0)


@pytest.mark.parametrize("pct", TOL)
@pytest.mark.parametrize("cfg", ((8, F.PAR_NONE), (8, F.PAR_EVEN), (9, F.PAR_ODD)), ids=("8N1", "8E1", "9O1"))
@pytest.mark.parametrize("B", (52, 104, 417))
def test_rx_baud_tolerance(B, cfg, pct):
    """A receiver sampling mid-bit survives half a bit of drift over a frame: 3% is the usual budget."""
    bits, parity = cfg
    b, nd = boot(B, bits=bits, parity=parity)
    m = (1 << bits) - 1
    words = [0x55 & m, 0xAA & m, m, 0, 1, 1 << (bits - 1), 0x7F & m, 0x80 & m] * 2
    seg = [s for w in words for s in frame(w, bits, parity)]
    b.devices.append(Script(play(1, seg, B * (1 + pct / 100), lead=20)))
    got = transfer(b, nd, n_rx=len(words) * (2 if bits > 8 else 1), limit=int(len(seg) * B * 1.1) + 20000)
    assert rx_words(got, bits) == words


@pytest.mark.parametrize("B", (52, 104, 417))
@pytest.mark.parametrize("frac", (0.1, 0.25, 0.4))
def test_rx_false_start_rejected(B, frac):
    """16550: a low pulse that is gone at mid-bit is not a start bit."""
    b, nd = boot(B)
    seg = [(0, frac), (1, 12.0)] * 3 + frame(0xA5)
    b.devices.append(Script(play(1, seg, B, lead=20)))
    assert transfer(b, nd, n_rx=1, limit=int(60 * B) + 20000) == [0xA5]
    b.run(12 * B)
    assert nd.chip.rxf.empty


@pytest.mark.parametrize("B", (52, 104, 417))
def test_rx_framing_error_then_recovery(B):
    """16550: a 0 stop bit is a framing error. The next good frame must be received."""
    b, nd = boot(B)
    seg = frame(0x5A, bad_stop=True) + [(1, 2.0)] + frame(0xC3)
    start = len(b.samples)
    b.devices.append(Script(play(1, seg, B, lead=20)))
    b.run(int(26 * B))
    d = sigrok.uart(b.samples[start:], 1, CLOCK_HZ // B)
    assert d["warn"], "the decoder must see the framing error, or the stimulus is not testing it"
    assert transfer(b, nd, n_rx=1, limit=4 * B + 20000) == [0xC3]


@pytest.mark.parametrize("parity", (F.PAR_EVEN, F.PAR_ODD, F.PAR_MARK, F.PAR_SPACE))
@pytest.mark.parametrize("B", (52, 104))
def test_rx_parity_error_detected(B, parity):
    b, nd = boot(B, parity=parity)
    seg = frame(0x35, 8, parity, bad_parity=True) + frame(0x36, 8, parity)
    start = len(b.samples)
    b.devices.append(Script(play(1, seg, B, lead=20)))
    assert transfer(b, nd, n_rx=1, limit=30 * B + 20000) == [0x36]
    assert sigrok.uart(b.samples[start:], 1, CLOCK_HZ // B, parity=PARITY[parity])["perr"]
    assert nd.peek(1)[0] == 1


@pytest.mark.parametrize("B", (52, 104, 417))
def test_rx_break_counted_once(B):
    """16550: a line low for longer than a whole frame is one break, however long it lasts."""
    b, nd = boot(B)
    nd.write(A_IRQ_EN, 0xFF)
    seg = [(0, 40.0), (1, 3.0)] + frame(0x77)
    start = len(b.samples)
    b.devices.append(Script(play(1, seg, B, lead=20)))
    assert transfer(b, nd, n_rx=1, limit=60 * B + 20000) == [0x77]
    assert nd.peek(2)[0] == 1, f"break count {nd.peek(2)[0]}"
    assert nd.irq() == 1
    assert sigrok.uart(b.samples[start:], 1, CLOCK_HZ // B)["brk"], "the decoder must call it a break"


@pytest.mark.parametrize("B", (52, 104, 417))
def test_tx_break_is_a_break(B):
    b, nd = boot(B)
    nd.dmem(3, [14])
    start = len(b.samples)
    nd.doorbell()
    b.run(20 * B)
    assert sigrok.uart(b.samples[start:], 0, CLOCK_HZ // B)["brk"]
    transfer(b, nd, [0x42])
    drain(b, nd, B)
    assert sigrok.uart(b.samples[start:], 0, CLOCK_HZ // B)["data"][-1:] == [0x42]


@pytest.mark.parametrize("B", (16, 24, 32, 52, 104))
def test_full_duplex(B):
    """Both directions at once, the sender out of phase with the chip's own bit clock."""
    b, nd = boot(B)
    r = random.Random(B)
    out = [r.randint(0, 255) for _ in range(64)]
    inn = [r.randint(0, 255) for _ in range(64)]
    seg = [s for w in inn for s in frame(w)]
    start = len(b.samples)
    b.devices.append(Script(play(1, seg, B, lead=7)))
    got = transfer(b, nd, out, n_rx=64, limit=90 * 12 * B)
    drain(b, nd, B)
    d = sigrok.uart(b.samples[start:], 0, CLOCK_HZ // B)
    assert got == inn and d["data"] == out and not d["warn"]
    assert not b.contention


@pytest.mark.parametrize("B", (2, 3, 4, 6, 8, 10, 12, 14))
def test_max_speed_report(B):
    """The fastest B: not a requirement of any standard, recorded in the doc."""
    pytest.importorskip("pytest")
    try:
        b, nd = boot(B)
    except AssertionError as e:
        pytest.skip(str(e))
    out = [0x55, 0xAA, 0x00, 0xFF, 0x12]
    start = len(b.samples)
    transfer(b, nd, out, limit=60000)
    drain(b, nd, B)
    d = sigrok.uart(b.samples[start:], 0, CLOCK_HZ // B)
    if d["data"] != out or d["warn"]:
        pytest.xfail(f"B={B} is too fast: decoded {d['data']} {d['warn'][:2]}")


def test_rx_overrun_reported():
    """16550: with the FIFO full, the character in the shift register is lost and overrun is reported."""
    B = 52
    b, nd = boot(B)
    data = list(range(0x30, 0x30 + 12))
    seg = [s for w in data for s in frame(w)]
    b.devices.append(Script(play(1, seg, B, lead=20)))
    b.run(14 * 12 * B)
    assert nd.chip.rxf.full
    overrun = (nd.read(A_STATUS) & 0x20) or nd.chip.rx.overrun or nd.chip.rxf.overrun
    assert overrun, "bytes were lost and nothing says so"
    got = transfer(b, nd, n_rx=8, limit=20000)
    assert got == data[:8], "the bytes kept must be the oldest, in order"


@pytest.mark.parametrize("skip", (17, 33, 52, 104, 200, 300, 400, 500))
def test_rx_joins_a_running_stream(skip):
    """Connected mid-frame: the receiver may misread what is on the line, but after one idle
    frame time it must be in step again."""
    B = 52
    b, nd = boot(B)
    seg = [s for w in (0xFF, 0x00, 0x55, 0xA5, 0x0F, 0x80) for s in frame(w)]
    cut, out = skip / B, []
    for lv, n in seg:
        if cut >= n:
            cut -= n
        else:
            out.append((lv, n - cut))
            cut = 0
    out += [(1, 12.0)] + frame(0x3C)
    b.devices.append(Script(play(1, out, B, lead=20)))
    b.run(int(sum(n for _, n in out) * B) + 40)
    got = []
    while not nd.chip.rxf.empty:
        got.append(nd.read(0x09))
    assert got and got[-1] == 0x3C, f"last byte {got}"


def noisy(frames, glitch, B, where, at=0.2):
    """Frames with a spike of `glitch` cycles, in the data bits or early in the start bit."""
    seg = []
    for f in frames:
        for k, (lv, n) in enumerate(f):
            if (where == "data" and 0 < k < len(f) - 1) or (where == "start" and k == 0):
                at = 0.5 if where == "data" else at
                first = n * at - glitch / B / 2
                seg += [(lv, first), (lv ^ 1, glitch / B), (lv, n - first - glitch / B)]
            else:
                seg.append((lv, n))
    return seg


@pytest.mark.parametrize("glitch", (1, 2, 4, 8))
def test_rx_noise_in_data_bits(glitch):
    """The vote of several samples per bit: a spike at mid-bit, where a 16550 would sample, is harmless."""
    B = 104
    b, nd = boot(B)
    b.devices.append(Script(play(1, noisy([frame(0xC5), frame(0x3A)], glitch, B, "data"), B, lead=20)))
    assert transfer(b, nd, n_rx=2, limit=30 * B + 20000) == [0xC5, 0x3A]


@pytest.mark.parametrize("glitch", (1, 2, 3))
def test_rx_noise_in_start_bit_with_filter(glitch):
    """With PIN_FILT on the RX pin, a spike of up to 3 cycles in the start bit changes nothing."""
    B = 104
    b = Bench(1)
    nd = b.nodes[0]
    nd.load(fw(B), own=owners(p0=OWN_TX), filt=0x02)
    b.run(200)
    b.devices.append(Script(play(1, noisy([frame(0xC5), frame(0x3A)], glitch, B, "start"), B, lead=20)))
    assert transfer(b, nd, n_rx=2, limit=30 * B + 20000) == [0xC5, 0x3A]


@pytest.mark.parametrize("at", (0.05, 0.1, 0.2, 0.3, 0.4, 0.45))
def test_rx_noise_in_start_bit_away_from_the_middle(at):
    """A 16550 only looks at the start bit at mid-bit, so a spike elsewhere does nothing to it. Here too,
    even with no filter and a spike of 8 cycles at a bit time of 104."""
    B = 104
    b, nd = boot(B)
    b.devices.append(Script(play(1, noisy([frame(0xC5), frame(0x3A)], 8, B, "start", at=at), B, lead=20)))
    assert transfer(b, nd, n_rx=2, limit=30 * B + 20000) == [0xC5, 0x3A]
