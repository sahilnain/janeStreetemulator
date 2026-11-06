# SPDX-License-Identifier: Apache-2.0
"""
test_spi: the chip model against the SPI mode definitions, judged by the sigrok spi decoder
plus spi_lib.check_frames (docs/protocol_conformance.md Section 3).
"""

import itertools
import random

import pe_fw as F
import pytest
import sigrok
from bench import OWN_TX, Bench, Script, fw_wait, owners
from pe_model.blocks import A_RXF, A_STATUS, A_TXF
from spi_lib import CS, MISO, MOSI, SCK, SpecSlave, check_frames, master_gen

SLAVE_MIN_LEAD = 6          # cycles, CS low to the first SCK edge (CPHA 1 is the tight case)
MODES = list(itertools.product((0, 1), (0, 1), (1, 0)))
IDS = [f"m{p << 1 | a}{'msb' if m else 'lsb'}" for p, a, m in MODES]


def mdata(n, seed, bits=8):
    r = random.Random(seed)
    m = (1 << bits) - 1
    head = [0, m, 0x5555 & m, 0xAAAA & m]
    return (head + [r.randint(0, m) for _ in range(n)])[:n]


def master(asm, own=None):
    b = Bench(1)
    nd = b.nodes[0]
    nd.load(asm, own=owners(p2=OWN_TX, p3=OWN_TX) if own is None else own)
    b.run(60)
    return b, nd


def dec(samples, cpol, cpha, msb, bits=8):
    d = sigrok.spi(samples, SCK, MOSI, MISO, CS, cpol, cpha, bool(msb), bits)
    assert not d["warn"], d["warn"][:3]
    return d


@pytest.mark.parametrize("B", (2, 3, 4, 5, 8, 16, 48))
@pytest.mark.parametrize("cpol,cpha,msb", MODES, ids=IDS)
def test_master_bytes(cpol, cpha, msb, B):
    b, nd = master(F.spi_master(B=B, cpol=cpol, cpha=cpha, msb=msb))
    out, back = mdata(24, B), mdata(24, B + 100)
    dev = SpecSlave(cpol, cpha, msb, tx=back)
    b.devices.append(dev)
    start = len(b.samples)
    nd.dmem(0, [len(out)] + out)
    nd.doorbell()
    fw_wait(b, nd)
    b.run(20)
    s = b.samples[start:]
    d = dec(s, cpol, cpha, msb)
    assert d["mosi"] == out and d["miso"] == back
    assert nd.peek(0x80, len(out)) == back and dev.rx == out
    assert check_frames(s, cpol, cpha, 16 * len(out), min_phase=B // 2, min_lead=B // 2, min_lag=B // 2) == 1


@pytest.mark.parametrize("bits", range(1, 17))
@pytest.mark.parametrize("cpol,cpha,msb", MODES, ids=IDS)
def test_master_every_word_size(cpol, cpha, msb, bits):
    B = 4
    b, nd = master(F.spi_master_words(B=B, cpol=cpol, cpha=cpha, msb=msb, bits=bits))
    out, back = mdata(8, bits, bits), mdata(8, bits + 50, bits)
    dev = SpecSlave(cpol, cpha, msb, bits, tx=back)
    b.devices.append(dev)
    start = len(b.samples)
    nd.dmem(0, [len(out)] + [x for w in out for x in (w & 0xFF, w >> 8)])
    nd.doorbell()
    fw_wait(b, nd)
    b.run(20)
    s = b.samples[start:]
    d = dec(s, cpol, cpha, msb, bits)
    assert d["mosi"] == out and d["miso"] == back
    got = nd.peek(0x80, 16)
    assert [got[2 * i] | got[2 * i + 1] << 8 for i in range(8)] == back
    check_frames(s, cpol, cpha, 2 * bits * 8, min_phase=B // 2, min_lead=B // 2, min_lag=B // 2)


@pytest.mark.parametrize("B", (3, 4, 8, 16))
@pytest.mark.parametrize("cpol,cpha,msb", MODES, ids=IDS)
def test_master_stream(cpol, cpha, msb, B):
    b, nd = master(F.spi_master_stream(B=B, cpol=cpol, cpha=cpha, msb=msb))
    n = 100
    out, back = mdata(n, B), mdata(n, B + 3)
    b.devices.append(SpecSlave(cpol, cpha, msb, tx=back))
    start = len(b.samples)
    nd.dmem(0, [n] + out)
    nd.doorbell()
    fw_wait(b, nd)
    b.run(40)
    s = b.samples[start:]
    d = dec(s, cpol, cpha, msb)
    assert d["mosi"] == out and d["miso"] == back
    assert nd.peek(0x7F) == [0] and nd.peek(0x80, n) == back
    check_frames(s, cpol, cpha, 16 * n, min_phase=B // 2, min_lead=B // 2, min_lag=B // 2)
    # no pause anywhere in the stream
    e = [t for t in range(1, len(s)) if (s[t] ^ s[t - 1]) & (1 << SCK)]
    assert max(y - x for x, y in zip(e, e[1:])) <= B - B // 2


class Three:
    """A 3-wire device: takes the data pin after the first shift edge of the read phase."""

    def __init__(self, cpol, cpha, msb, B, rd):
        self.cpol, self.cpha, self.msb, self.B = cpol, cpha, msb, B
        self.rx, self.tx, self.p_sck, self.p_cs = [], list(rd), cpol, 1
        self.nin, self.sh, self.nout, self.cur, self.on, self.out, self.at = 0, 0, 8, 0, False, 1, None

    def step(self, line, t):
        sck, cs, dio = (line >> SCK) & 1, (line >> CS) & 1, (line >> MOSI) & 1
        if not cs and sck != self.p_sck:
            sample = (sck != self.cpol) != bool(self.cpha)
            if sample and len(self.rx) < 4:
                self.sh = (self.sh << 1 | dio) if self.msb else (self.sh | dio << self.nin)
                self.nin += 1
                if self.nin == 8:
                    self.rx.append(self.sh)
                    self.nin, self.sh = 0, 0
            elif not sample and len(self.rx) == 4:
                if self.nout == 8:
                    self.cur, self.nout = (self.tx.pop(0) if self.tx else 0xFF), 0
                self.out = (self.cur >> (7 - self.nout if self.msb else self.nout)) & 1
                self.nout += 1
                if self.at is None:
                    # CPHA 0: after the master let go (8 cycles). CPHA 1: before its sampling edge.
                        self.at = t + (max(1, self.B // 2 - 2) if self.cpha else 8)
        if self.at is not None and t >= self.at:
            self.on = True
        self.p_sck, self.p_cs = sck, cs
        return (1 << MOSI, self.out << MOSI) if self.on and not cs else (0, 0)



@pytest.mark.parametrize("B", (8, 16, 32))
@pytest.mark.parametrize("cpol,cpha,msb", MODES, ids=IDS)
def test_master_3wire(cpol, cpha, msb, B):
    b, nd = master(F.spi_master_3wire(B=B, cpol=cpol, cpha=cpha, msb=msb), owners(p2=OWN_TX, p3=OWN_TX))
    wr, rd = mdata(4, B), mdata(4, B + 7)

    dev = Three(cpol, cpha, msb, B, rd)
    b.devices.append(dev)
    start = len(b.samples)
    nd.dmem(0, [4, 4] + wr)
    nd.doorbell()
    fw_wait(b, nd)
    b.run(20)
    s = b.samples[start:]
    assert not b.contention, b.contention[:3]
    d = sigrok.spi(s, SCK, MOSI, None, CS, cpol, cpha, bool(msb), 8)
    assert d["mosi"][:4] == wr and dev.rx == wr
    assert d["mosi"][4:] == rd and nd.peek(0x80, 4) == rd


def slave(asm, miso=MISO, tx=()):
    b = Bench(1)
    nd = b.nodes[0]
    for w in tx:                # before the firmware starts: it takes its first word at once
        nd.write(A_TXF, w)
    nd.load(asm, own=owners(**{f"p{miso}": OWN_TX}))
    b.run(60)
    return b, nd


def pump(b, nd, scr, tx, want):
    """Host: keep TXF topped up and collect RXF until the master has finished."""
    got = []
    while not scr.done or (want and len(got) < want and not nd.chip.rxf.empty):
        st = nd.read(A_STATUS)
        if tx and not st & 0x02:
            nd.write(A_TXF, tx.pop(0))
        elif not st & 0x04:
            got.append(nd.read(A_RXF))
        else:
            b.run(2)
    while not nd.chip.rxf.empty:
        got.append(nd.read(A_RXF))
    return got


@pytest.mark.parametrize("bits", (1, 2, 3, 4, 5, 6, 7, 8))
@pytest.mark.parametrize("cpol,cpha,msb", MODES, ids=IDS)
def test_slave_every_word_size(cpol, cpha, msb, bits):
    b, nd = slave(F.spi_slave(cpol, cpha, msb, bits=bits % 8))
    down, up = mdata(16, bits, bits), mdata(16, bits + 9, bits)
    h = 24
    for w in up[:8]:
        nd.write(A_TXF, w)
    rest = up[8:]
    got = []
    scr = Script(None)
    scr.gen = master_gen(scr, [down], bits, h, cpol, cpha, msb, got=got, lead=48)
    start = len(b.samples)
    b.devices.append(scr)
    rx = pump(b, nd, scr, rest, 16)
    s = b.samples[start:]
    d = dec(s, cpol, cpha, msb, bits)
    assert d["mosi"] == down
    assert d["miso"] == up and got == up
    assert rx[:16] == down
    check_frames(s, cpol, cpha, 2 * bits * 16, data=(MISO,))


@pytest.mark.parametrize("h", (4, 5, 6, 7, 8, 12, 16))
@pytest.mark.parametrize("cpol,cpha,msb", MODES, ids=IDS)
def test_slave_speed(cpol, cpha, msb, h):
    if not cpha and h < 6 or cpha and h < 4:
        pytest.skip("below the documented limit")
    b, nd = slave(F.spi_slave(cpol, cpha, msb))
    down, up = mdata(8, h), mdata(8, h + 9)
    for w in up:
        nd.write(A_TXF, w)
    got = []
    scr = Script(None)
    scr.gen = master_gen(scr, [down], 8, h, cpol, cpha, msb, got=got, lead=48)
    start = len(b.samples)
    b.devices.append(scr)
    rx = pump(b, nd, scr, [], 8)
    s = b.samples[start:]
    d = dec(s, cpol, cpha, msb)
    assert d["mosi"] == down and d["miso"] == up and got == up and rx[:8] == down
    check_frames(s, cpol, cpha, 128, data=(MISO,))


@pytest.mark.parametrize("bits", (9, 10, 11, 12, 13, 14, 15, 16))
@pytest.mark.parametrize("cpol,cpha,msb", MODES, ids=IDS)
def test_slave_long_words(cpol, cpha, msb, bits):
    down, up = mdata(4, bits, bits), mdata(4, bits + 9, bits)
    tx = [x for w in up for x in (w & 0xFF, w >> 8)]
    b, nd = slave(F.spi_slave_words(bits, cpol, cpha, msb), tx=tx[:8])
    got = []
    scr = Script(None)
    scr.gen = master_gen(scr, [down], bits, 16, cpol, cpha, msb, got=got, lead=32)
    start = len(b.samples)
    b.devices.append(scr)
    rx = pump(b, nd, scr, [], 8)
    s = b.samples[start:]
    d = dec(s, cpol, cpha, msb, bits)
    assert d["mosi"] == down and d["miso"] == up and got == up
    assert [rx[2 * i] | rx[2 * i + 1] << 8 for i in range(4)] == down
    check_frames(s, cpol, cpha, 2 * bits * 4, data=(MISO,))


@pytest.mark.parametrize("cpol,cpha,msb", MODES, ids=IDS)
def test_slave_3wire(cpol, cpha, msb):
    b, nd = slave(F.spi_slave_3wire(cpol, cpha, msb), miso=MOSI)
    cmd = 0x80                                  # the value of the byte, whatever the bit order
    up = [0xA5, 0x3C, 0x0F]
    for w in up:
        nd.write(A_TXF, w)
    got = []
    h = 24
    scr = Script(None)

    def gen():
        yield {CS: 1, SCK: cpol, MOSI: 0}
        yield 32
        yield {CS: 0}
        yield 48
        for k in range(8):                     # command, driven by the master
            i = 7 - k if msb else k
            if not cpha:
                yield {MOSI: (cmd >> i) & 1}
                yield h - 1
                yield {SCK: 1 - cpol}
                yield h - 1
                yield {SCK: cpol}
            else:
                yield {SCK: 1 - cpol, MOSI: (cmd >> i) & 1}
                yield h - 1
                yield {SCK: cpol}
                yield h - 1
        yield {MOSI: None}                     # released: the chip answers
        yield h
        for _ in up:
            v = 0
            for k in range(8):
                i = 7 - k if msb else k
                if not cpha:
                    yield h - 1
                    v |= ((scr.line >> MOSI) & 1) << i
                    yield {SCK: 1 - cpol}
                    yield h - 1
                    yield {SCK: cpol}
                else:
                    yield {SCK: 1 - cpol}
                    yield h - 1
                    v |= ((scr.line >> MOSI) & 1) << i
                    yield {SCK: cpol}
                    yield h - 1
            got.append(v)
        yield h
        yield {CS: 1}
        yield 40

    scr.gen = gen()
    b.devices.append(scr)
    pump(b, nd, scr, [], 1)
    assert not b.contention, f"both drove the data pin: {b.contention[:3]}"
    assert got == up


@pytest.mark.parametrize("cpol,cpha", ((0, 0), (0, 1), (1, 0), (1, 1)))
def test_slave_releases_miso_when_not_selected(cpol, cpha):
    """SPI slaves share MISO, so a slave that is not selected must leave it alone."""
    b, nd = slave(F.spi_slave(cpol, cpha))
    for w in (1, 2, 3, 4):
        nd.write(A_TXF, w)
    b.devices.append(Script(iter(({CS: 1, SCK: cpol, MOSI: 0}, 100))))
    b.run(150)
    assert not (nd.outs["uio_oe"] >> MISO) & 1, "MISO driven before any select"
    scr = Script(None)
    scr.gen = master_gen(scr, [[0x12, 0x34]], 8, 16, cpol, cpha, lead=48, gap=40)
    b.devices = [scr]
    pump(b, nd, scr, [], 0)
    b.run(10)
    assert not (nd.outs["uio_oe"] >> MISO) & 1, "MISO still driven after CS went high"


@pytest.mark.parametrize("cut", (1, 3, 5, 7))
@pytest.mark.parametrize("cpol,cpha", ((0, 0), (0, 1), (1, 0), (1, 1)))
def test_slave_cs_abort_resyncs(cpol, cpha, cut):
    """CS high in the middle of a word ends the frame: the next frame starts from bit 0."""
    b, nd = slave(F.spi_slave(cpol, cpha))
    for w in (0xFF, 0xA5):
        nd.write(A_TXF, w)
    got = []
    scr = Script(None)

    def gen():
        yield from master_gen(scr, [[0xFF]], 8, 16, cpol, cpha, lead=48, abort_after=cut)
        yield from master_gen(scr, [[0x5A]], 8, 16, cpol, cpha, lead=48, got=got)

    scr.gen = gen()
    b.devices.append(scr)
    rx = pump(b, nd, scr, [], 1)
    assert rx and rx[-1] == 0x5A, f"after an aborted word the slave read {rx}"
    assert got[-1] == 0xA5, f"after an aborted word the slave sent {got}"


@pytest.mark.parametrize("B", (4, 16))
def test_two_chips(B):
    """Chip as master, chip as slave, both modes of phase: the same trace for the decoder."""
    for cpol, cpha in ((0, 0), (0, 1), (1, 0), (1, 1)):
        b = Bench(2)
        m, sl = b.nodes
        m.load(F.spi_master(B=max(B, 40), cpol=cpol, cpha=cpha), own=owners(p2=OWN_TX, p3=OWN_TX))
        sl.load(F.spi_slave(cpol, cpha), own=owners(p4=OWN_TX))
        b.run(80)
        out, back = mdata(8, B), mdata(8, B + 1)
        for w in back:
            sl.write(A_TXF, w)
        start = len(b.samples)
        m.dmem(0, [8] + out)
        m.doorbell()
        fw_wait(b, m)
        b.run(10)
        d = dec(b.samples[start:], cpol, cpha, 1)
        assert d["mosi"] == out and d["miso"] == back and m.peek(0x80, 8) == back


@pytest.mark.parametrize("lead", (4, 6, 8, 10, 12, 14, 16, 20))
@pytest.mark.parametrize("cpol,cpha", ((0, 0), (0, 1), (1, 0), (1, 1)))
def test_slave_shortest_cs_lead(cpol, cpha, lead):
    """CS low to the first SCK edge: the first bit must be right. The limit goes in the doc."""
    b, nd = slave(F.spi_slave(cpol, cpha))
    up = [0xA5, 0x3C]
    for w in up:
        nd.write(A_TXF, w)
    got = []
    scr = Script(None)
    scr.gen = master_gen(scr, [[0x5A, 0xC3]], 8, 8, cpol, cpha, lead=lead, got=got)
    b.devices.append(scr)
    rx = pump(b, nd, scr, [], 2)
    if lead >= SLAVE_MIN_LEAD:
        assert got == up and rx[:2] == [0x5A, 0xC3], f"lead {lead}: sent {got}, read {rx}"
