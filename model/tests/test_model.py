# SPDX-License-Identifier: Apache-2.0
"""
test_model: checks of the Python reference model against the spec, with no RTL.

What is checked
---------------
These tests check that the model behaves as docs/spec/ says, so that agreement with the
RTL in test/test_lockstep.py means something.
1. test_registers_reset    ID, VERSION, RUN and the pin reset state over the host bus.
2. test_instruction_timing 2 cycles per instruction; dly and `delay` rounded up to slots.
3. test_tx_times           TX bit i is on the pad during S+1+iB ... S+(i+1)B.
4. test_tx_nbits_msb       word length and bit order on the pad.
5. test_rx_vote_majority   the vote takes the majority of the samples between K and N-K.
6. test_uart_loopback      pe_fw.uart, P0 looped to P1: bytes arrive and the line is on the bit grid.
7. test_fifo_arbitration   overrun flags, as blocks.md Section 8.
8. test_owner_rule         a controller write to a busy unit's pin is dropped and sets owner_err.
9. test_crc_usb            the CRC unit gives the USB residuals.
"""

import random

import pe_fw
import pe_isa as I
import uart
from pe_isa import Asm
from pe_model import Chip
from pe_model.blocks import A_ERR_CLR, A_FIFO_CNT, A_ID, A_RUN, A_RXF, A_STATUS, A_TXF, A_VERSION
from pe_model.host import ModelHost



def p0_to_p1(o):
    line = (o["uio_out"] & o["uio_oe"]) | (0xFF & ~o["uio_oe"])
    return (line & ~2) | ((line & 1) << 1)


def host(pins_in=None):
    h = ModelHost(Chip(), pins_in)
    h.reset()
    return h


def run(words, owners=0, pins_in=None, cycles=200, pc=0):
    h = host(pins_in)
    h.load_program(words, pc)
    h.owners(owners)
    h.start(pc)
    t0 = len(h.trace)
    h.cycle(cycles)
    return h, h.trace[t0:]


def pin(trace, p):
    return [((o["uio_out"] & o["uio_oe"]) | (0xFF & ~o["uio_oe"])) >> p & 1 for o in trace]


def edges(levels):
    return [i for i in range(1, len(levels)) if levels[i] != levels[i - 1]]


def test_registers_reset():
    h = host()
    assert h.read(A_ID) == 0x50 and h.read(A_VERSION) == 0x02 and h.read(A_RUN) == 0
    assert h.read(A_STATUS) == 0x05, "both FIFOs empty"
    o = h.chip.outputs()
    assert o["uio_oe"] == 0


def test_instruction_timing():
    for d, cost in ((0, 2), (1, 2), (2, 4), (3, 4), (7, 8)):
        a = Asm().setoe(0, 1).L("l").set(0, I.SRC_0, dly=d).set(0, I.SRC_1, dly=d).jmp("l", dly=0)
        _, tr = run(a.words(), cycles=120)
        low = pin(tr, 0)
        e = edges(low)[2:6]
        assert e[1] - e[0] in (cost, cost + 2) and e[3] - e[2] in (cost, cost + 2), (d, e)
        first = sorted({y - x for x, y in zip(e, e[1:])})
        assert first[0] == cost, (d, first)
    a = Asm().setoe(0, 1).set(0, I.SRC_0).delay(100).set(0, I.SRC_1).L("e").jmp("e")
    _, tr = run(a.words(), cycles=200)
    e = edges(pin(tr, 0))
    assert e[1] - e[0] == 2 + 102, "set (2 cycles), then delay 100: E + 2*ceil(101/2) = 102"


def tx_prog(B, data, **frame):
    a = Asm()
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(0)).ucfg(I.TX, I.TX_BIT, B).ucfg(I.TX, I.TX_FRAME, I.tx_frame(**frame))
    a.setoe(0, 1).ustart(I.M_TX, data).uwait(I.M_TX).L("e").jmp("e")
    return a.words()


def test_tx_times():
    B = 7
    _, tr = run(tx_prog(B, 0x55), owners=1, cycles=120)
    e = edges(pin(tr, 0))
    assert [y - x for x, y in zip(e, e[1:])][:6] == [B] * 6, e


def test_tx_nbits_msb():
    B = 4
    for n, msb, word in ((3, 0, 0b101), (12, 1, 0xA5C), (16, 0, 0x8001), (5, 1, 0b10011)):
        _, tr = run(tx_prog(B, word, msb=msb, nbits=n), owners=1, cycles=30 + n * B)
        lv = pin(tr, 0)
        start = next(i for i, o in enumerate(tr) if o["uio_oe"] & 1)
        # the unit starts 2 cycles after setoe (one instruction); bit i runs from there
        got = [lv[start + 3 + i * B] for i in range(n)]
        want = [(word >> (n - 1 - i if msb else i)) & 1 for i in range(n)]
        assert got == want, (n, msb, got, want)


def test_rx_vote_majority():
    """5 samples, K = 1: the 3 in the middle count. The line is high only for the first and last."""
    a = Asm()
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(1)).ucfg(I.RX, I.RX_BIT, 20).ucfg(I.RX, I.RX_VOTE, I.rx_vote(5, 1))
    a.ucfg(I.RX, I.RX_STEP, 3).ucfg(I.RX, I.RX_FIRST, 2).ucfg(I.RX, I.RX_FRAME, I.rx_frame(hold=1, nbits=1))
    a.ustart(I.M_RX).uwait(I.M_RX).urd(0, I.RX, I.RX_DATA).fput(0, I.FIFO_RX).L("e").jmp("e")
    t = {"n": 0}

    def line(o):
        t["n"] += 1
        return 0xFF

    h, _ = run(a.words(), pins_in=line, cycles=120)
    assert h.read(A_RXF) == 1, "an all-high line votes 1"


def test_uart_loopback():
    B = 24
    h = host(p0_to_p1)
    h.load_program(pe_fw.uart(B=B, N=5, K=1, S=4).words())
    h.owners(1)
    h.start()
    h.cycle(60)
    data = [0x00, 0xFF, 0x5A, random.getrandbits(8)]
    t0 = len(h.trace)
    got = []
    for d in data:
        h.write(A_TXF, d)
        h.cycle(12 * B)
        got.append(h.read(A_RXF))
    assert got == data
    assert uart.decode(pin(h.trace[t0:], 0), B) == data, "the line is cycle-exact on the bit grid"


def test_fifo_arbitration():
    h = host()
    for i in range(9):
        h.write(A_TXF, i)
    st = h.read(A_STATUS)
    assert st & 0x02 and st & 0x10, f"TXF full and overrun: {st:02x}"
    assert h.read(A_FIFO_CNT) == 0x08
    h.write(A_ERR_CLR, 0x10)
    assert not h.read(A_STATUS) & 0x10


def test_owner_rule():
    a = Asm()
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(0)).ucfg(I.TX, I.TX_BIT, 40).setoe(0, 1)
    a.ustart(I.M_TX, 0xFF).set(0, I.SRC_0).L("e").jmp("e")
    h, tr = run(a.words(), owners=1, cycles=100)
    assert h.read(A_STATUS) & 0x40, "owner_err"
    assert all(pin(tr, 0)), "the dropped write never reaches the pad"


def _crc_feed(poly, init, bits):
    c = init
    for b in bits:
        c = ((c << 1) ^ (poly if ((c >> 15) ^ b) & 1 else 0)) & 0xFFFF
    return c


def test_crc_usb():
    from pe_model.blocks import Crc

    u = Crc()
    u.clock(0, 1, I.CRC_CTRL, I.crc_ctrl(I.CRC_RX), 0, 0, 0, 0, 0)
    u.clock(0, 1, I.CRC_POLY, 0x8005, 0, 0, 0, 0, 0)
    u.clock(0, 1, I.CRC_INIT, 0xFFFF, 0, 0, 0, 0, 0)
    data = [random.getrandbits(1) for _ in range(40)]
    for b in data:
        u.clock(0, 0, 0, 0, 0, 0, 1, b, 0)
    assert u.crc == _crc_feed(0x8005, 0xFFFF, data)
    v = u.value
    for i in range(16):
        u.clock(0, 0, 0, 0, 0, 0, 1, (v >> (15 - i)) & 1, 0)
    assert u.crc == 0x800D, f"residual {u.crc:04x}"
