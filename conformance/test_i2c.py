# SPDX-License-Identifier: Apache-2.0
"""
test_i2c: the chip model against NXP UM10204 Rev. 7, judged by the sigrok i2c decoder and the limits of
Tables 11 and 13, with ideal edges and with the slowest edges the standard allows.
Sections: docs/protocol_conformance.md Section 4.
"""

import random

import i2c_lib as L
import pe_fw as F
import pytest
import sigrok
from bench import OWN_RX, OWN_TX, Bench, Script, fw_wait, owners
from i2c_lib import SCL, SDA, Controller, Target, expect_read, expect_write, timing

MODES = ("SM", "FM", "FM+")
FILT = 0xC0     # the input filter on SCL and SDA, as I2C firmware should have it
ME = 0x42


def master(B=None, mode="FM", edges=None, target=None, **kw):
    B = B or L.MASTER_B[mode]
    b = Bench(1, edges=edges)
    nd = b.nodes[0]
    filt = 0 if kw.get("hs_B") else FILT         # Table 12: 10 ns in Hs mode, under one cycle
    nd.load(F.i2c_master(B=B, scl=SCL, sda=SDA, **kw), own=owners(p6=OWN_TX, p7=OWN_RX), od=0xC0, filt=filt)
    if target is not None:
        b.devices.append(target)
    b.run(60)
    return b, nd


def request(b, nd, addr, wdata=(), nread=0):
    nd.dmem(0, [addr, len(wdata), nread] + list(wdata))
    nd.doorbell()
    fw_wait(b, nd, limit=3_000_000)
    b.run(200)
    return nd.peek(0x7F)[0], nd.peek(0x80, nread)


def events(s):
    d = sigrok.i2c(s, SCL, SDA)
    assert not d["warn"], d["warn"][:3]
    return d["ev"]


@pytest.mark.parametrize("slow", (False, True), ids=("ideal", "slowest"))
@pytest.mark.parametrize("mode", MODES)
def test_master_write_then_read(mode, slow):
    t = Target()
    b, nd = master(mode=mode, edges=L.slow_edges(mode) if slow else None, target=t)
    s0 = len(b.samples)
    data = [0xA5, 0x00, 0xFF, 0x3C]
    assert request(b, nd, 0x50, [0x20] + data)[0] == 0
    assert t.mem[0x20:0x24] == data
    st, got = request(b, nd, 0x50, [0x20], 4)
    assert st == 0 and got == data
    s = b.samples[s0:]
    assert events(s) == (expect_write(0x50, [0x20] + data) + expect_write(0x50, [0x20], stop=False)
                         + expect_read(0x50, data, start="Sr"))
    assert not (v := timing(s, mode)), v
    assert not b.contention


@pytest.mark.parametrize("mode", MODES)
def test_master_data_hold(mode):
    """Table 11 note 2: the controller's SDA must not move until SCL is below its low level, so at
    least the fall time after it pulled SCL down. Only the chip's own SDA edges count here."""
    b, nd = master(mode=mode, target=Target())
    nd.dmem(0, [0x50, 3, 0, 0x20, 0xAA, 0x55])
    nd.doorbell()
    worst, fall, ps, pd = None, None, 1, nd.outs["uio_oe"] >> SDA & 1
    for _ in range(200000):
        b.tick()
        o = nd.outs["uio_oe"]
        c, d = (o >> SCL) & 1, (o >> SDA) & 1      # open drain: OE on means driven low
        if c and not ps:
            fall = 0
        elif fall is not None:
            fall += 1
        if d != pd and c and fall is not None:
            worst = fall if worst is None else min(worst, fall)
        ps, pd = c, d
        if nd.chip.regs.st2 & 0x10:
            break
    assert worst is not None and worst * L.NS >= L.LIMITS[mode]["tf"], f"hold {worst * L.NS:.0f} ns"


def test_master_every_7bit_address():
    """Every address: only the device's own is acknowledged, the rest give status 1 and a STOP."""
    t = Target(0x50)
    b, nd = master(mode="FM+", target=t)
    for a in range(128):
        s0 = len(b.samples)
        st, _ = request(b, nd, a, [1])
        ev = events(b.samples[s0:])
        if a == 0x50:
            assert st == 0 and ev == expect_write(a, [1])
        else:
            assert st == 1 and ev == ["S", ("AW", a), "N", "P"], (a, ev)


def test_master_every_data_byte():
    t = Target()
    b, nd = master(mode="FM+", target=t)
    s0 = len(b.samples)
    for hi in range(0, 256, 64):
        chunk = list(range(hi, hi + 64))
        assert request(b, nd, 0x50, [hi] + chunk)[0] == 0
        assert t.mem[hi:hi + 64] == chunk
        st, got = request(b, nd, 0x50, [hi], 64)
        assert st == 0 and got == chunk
    assert not (v := timing(b.samples[s0:], "FM+")), v


@pytest.mark.parametrize("after", (1, 2, 3))
def test_master_data_nack(after):
    """3.1.6: on NACK the controller ends with STOP, and the status says so."""
    t = Target(nack_after=after)
    b, nd = master(mode="FM", target=t)
    s0 = len(b.samples)
    assert request(b, nd, 0x50, [1, 2, 3, 4, 5])[0] == 2
    ev = events(b.samples[s0:])
    assert ev[-2:] == ["N", "P"] and ev.count("A") == after + 1


@pytest.mark.parametrize("stretch", ({"addr": 500}, {"read": 300}, {"bit": 40}, {"bit": 700}, {"addr": 20000}),
                         ids=("addr500", "read300", "bit40", "bit700", "addr20k"))
@pytest.mark.parametrize("mode", MODES)
def test_master_clock_stretching(mode, stretch):
    """3.1.9: at byte level and at bit level the controller waits for SCL."""
    t = Target(stretch=stretch)
    t.mem[0x10:0x13] = [0x11, 0x22, 0x33]
    b, nd = master(mode=mode, target=t)
    s0 = len(b.samples)
    st, got = request(b, nd, 0x50, [0x10], 3)
    assert st == 0 and got == [0x11, 0x22, 0x33]
    assert t.stretched > 0
    s = b.samples[s0:]
    assert events(s) == expect_write(0x50, [0x10], stop=False) + expect_read(0x50, got, start="Sr")
    v = timing(s, mode, check=("high", "su_sta", "su_sto", "su_dat", "hd_sta", "buf", "fscl"))
    assert not v, v


@pytest.mark.parametrize("A", (0x000, 0x0FF, 0x155, 0x2AA, 0x3FF))
def test_master_10bit(A):
    """3.1.11: 11110 A9 A8 W, A7..A0, then Sr and 11110 A9 A8 R."""
    t = Target(addr10=A)
    b, nd = master(mode="FM", target=t, addr10=True)
    s0 = len(b.samples)
    nd.dmem(0x7E, [A >> 8])
    st, got = request(b, nd, A & 0xFF, [0x20, 0x77], 0)
    assert st == 0 and t.mem[0x20] == 0x77
    st, got = request(b, nd, A & 0xFF, [0x20], 1)
    assert st == 0 and got == [0x77]
    ev = events(b.samples[s0:])
    hi = 0x78 | (A >> 8)
    assert ev[:4] == ["S", ("AW", hi), "A", ("DW", A & 0xFF)], ev[:4]
    assert ("AR", hi) in ev


def test_master_hs_mode():
    """5.3.2: F/S START, master code at F/S speed with NACK, Sr, then Hs. Timing per Table 13."""
    t = Target()
    b, nd = master(B=126, target=t, hs_B=16, sdly=2)
    s0 = len(b.samples)
    st, _ = request(b, nd, 0x50, [0x20, 0xA5, 0x5A])
    assert st == 0 and t.mem[0x20:0x22] == [0xA5, 0x5A]
    s = b.samples[s0:]
    d = sigrok.i2c(s, SCL, SDA)
    assert not d["warn"]
    assert d["ev"][:4] == ["S", ("AW", 0x04), "N", "Sr"], d["ev"][:5]    # 0000 1000 >> 1
    assert d["ev"][4:] == expect_write(0x50, [0x20, 0xA5, 0x5A])[1:]
    sr = [a.start for a in d["ann"] if a.cls == "repeat-start"][0]
    fall = next(t for t in range(sr, len(s)) if not (s[t] >> SCL) & 1)
    assert not (v := timing(s, "FM", stop=fall + 4, check=("fscl", "low", "high", "su_sta", "hd_sta"))), v
    v = timing(s, "HS", start=sr - 2, check=("fscl", "high", "su_sta", "hd_sta", "su_dat", "su_sto"))
    assert not v, v


def two_masters(BA=64, BB=64):
    b = Bench(2)
    A, Bb = b.nodes
    for nd, B in ((A, BA), (Bb, BB)):
        nd.load(F.i2c_master(B=B, scl=SCL, sda=SDA, arb=True), own=owners(p6=OWN_TX, p7=OWN_RX), od=0xC0, filt=FILT)
    t = Target(0x50)
    b.devices.append(t)
    b.run(60)
    return b, A, Bb, t


def both_go(b, A, Bb, ra, rb, skew=0):
    A.dmem(0, ra)
    Bb.dmem(0, rb)
    A.chip.ctrl.db = 1
    b.run(skew)
    Bb.chip.ctrl.db = 1
    b.until(lambda: A.chip.regs.st2 & 0x10 and Bb.chip.regs.st2 & 0x10, limit=3_000_000, what="both masters done")
    A.write(0x0E, 0x10)
    Bb.write(0x0E, 0x10)
    b.run(200)


def both_go_sync(b, A, Bb, BA, BB, ra, rb):
    """The two STARTs come together: each waits its own bus free time, so the slower one goes first."""
    d = abs(BA - BB) // 2
    if BA >= BB:
        both_go(b, A, Bb, ra, rb, d)
    else:
        both_go(b, Bb, A, rb, ra, d)


@pytest.mark.parametrize("skew", (0, 1, 2, 3, 5, 8))
def test_arbitration_on_address(skew):
    """3.1.8: the one sending 1 where the other sends 0 loses, stops driving, sends no STOP."""
    b, A, Bb, t = two_masters()
    s0 = len(b.samples)
    both_go(b, A, Bb, [0x50, 2, 0, 0x10, 0xAA], [0x58, 2, 0, 0x10, 0xBB], skew)
    assert A.peek(0x7F)[0] == 0 and Bb.peek(0x7F)[0] == 3, (A.peek(0x7F), Bb.peek(0x7F))
    assert t.mem[0x10] == 0xAA
    assert events(b.samples[s0:]) == expect_write(0x50, [0x10, 0xAA])


@pytest.mark.parametrize("skew", (0, 2, 5))
def test_arbitration_on_data(skew):
    """Same address: arbitration goes on through the data bytes."""
    b, A, Bb, t = two_masters()
    s0 = len(b.samples)
    both_go(b, A, Bb, [0x50, 2, 0, 0x10, 0x55], [0x50, 2, 0, 0x10, 0x57], skew)
    assert A.peek(0x7F)[0] == 0 and Bb.peek(0x7F)[0] == 3
    assert t.mem[0x10] == 0x55
    assert events(b.samples[s0:]) == expect_write(0x50, [0x10, 0x55])


@pytest.mark.parametrize("BA,BB", ((64, 96), (96, 64), (64, 200), (200, 64)))
def test_clock_synchronization(BA, BB):
    """3.1.7: masters of different speeds make one clock: the longest low, the shortest high."""
    b, A, Bb, t = two_masters(BA, BB)
    s0 = len(b.samples)
    both_go_sync(b, A, Bb, BA, BB, [0x50, 2, 0, 0x10, 0xAA], [0x50, 2, 0, 0x10, 0xAA])
    assert A.peek(0x7F)[0] == 0 and Bb.peek(0x7F)[0] == 0, (A.peek(0x7F), Bb.peek(0x7F))
    assert t.mem[0x10] == 0xAA
    s = b.samples[s0:]
    assert events(s) == expect_write(0x50, [0x10, 0xAA]), "not one transfer: they did not start together"
    # the bus clock: low phases at least the longer one, high phases at most the shorter one + latency
    e = [i for i in range(1, len(s)) if (s[i] ^ s[i - 1]) >> SCL & 1]
    lows = [y - x for x, y in zip(e, e[1:]) if not (s[x] >> SCL) & 1][1:-1]
    assert min(lows) >= max(BA, BB) // 2 - 4, f"SCL low {min(lows)}"


@pytest.mark.parametrize("BA,BB", ((64, 96), (96, 64)))
def test_clock_synchronization_read(BA, BB):
    """Two controllers reading the same bytes keep one clock, and both read them."""
    b, A, Bb, t = two_masters(BA, BB)
    t.mem[0x30:0x33] = [0xA5, 0x5A, 0xFF]
    both_go_sync(b, A, Bb, BA, BB, [0x50, 1, 3, 0x30], [0x50, 1, 3, 0x30])
    assert A.peek(0x7F)[0] == 0 and Bb.peek(0x7F)[0] == 0, (A.peek(0x7F), Bb.peek(0x7F))
    assert A.peek(0x80, 3) == [0xA5, 0x5A, 0xFF] and Bb.peek(0x80, 3) == [0xA5, 0x5A, 0xFF]


def test_master_waits_for_a_free_bus():
    """3.1.8: a controller may start only if the bus is free: from a START until the STOP, it is busy,
    even in a long SCL high phase of a slower controller."""
    t = Target(0x50)
    b, nd = master(B=48, target=t, arb=True)
    scr = Script(None)
    other = Controller(scr, "SM")

    def gen():
        yield from other.start()
        yield from other.wbyte(0x50 << 1)
        for d in (0x10, 0xFF, 0xFF, 0xFF, 0xFF):
            yield from other.wbyte(d)
        yield from other.stop()

    scr.gen = gen()
    s0 = len(b.samples)
    b.devices.append(scr)
    b.run(20000)                     # the slow controller is in the middle of its transfer
    assert not scr.done
    nd.dmem(0, [0x50, 2, 0, 0x20, 0xCC])
    nd.doorbell()
    fw_wait(b, nd, limit=3_000_000)
    b.run(200)
    ev = events(b.samples[s0:])
    n = ev.index("P")
    assert ev[:n + 1] == expect_write(0x50, [0x10, 0xFF, 0xFF, 0xFF, 0xFF]), "the chip broke into a transfer"
    assert ev[n + 1:] == expect_write(0x50, [0x20, 0xCC])
    assert not (v := timing(b.samples[s0:], "FM+", check=("buf",))), v


# -------------------------------------------------------------- chip as target
def target_chip(edges=None, **kw):
    b = Bench(1, edges=edges)
    nd = b.nodes[0]
    filt = kw.pop("filt", FILT)
    nd.load(F.i2c_slave(addr=kw.pop("addr", ME), scl=SCL, sda=SDA, **kw), own=owners(p7=OWN_TX), od=0xC0, filt=filt)
    b.run(60)
    nd.dmem(0, [0] * 256)
    return b, nd


def run_ctrl(b, gen_factory, mode="FM", h=None):
    scr = Script(None)
    c = Controller(scr, mode, h=h)
    out = {}
    scr.gen = gen_factory(c, out)
    b.devices.append(scr)
    b.until(lambda: scr.done, limit=4_000_000, what="controller")
    b.devices.remove(scr)
    b.run(20)
    return out


def write_tx(addr, data):
    def f(c, out):
        yield from c.start()
        out["acks"] = [(yield from c.wbyte(addr << 1))]
        for d in data:
            out["acks"].append((yield from c.wbyte(d)))
        yield from c.stop()
    return f


def read_tx(addr, ptr, n):
    def f(c, out):
        yield from c.start()
        a0 = yield from c.wbyte(addr << 1)
        a1 = yield from c.wbyte(ptr)
        yield from c.restart()
        a2 = yield from c.wbyte(addr << 1 | 1)
        out["acks"] = [a0, a1, a2]
        out["data"] = []
        for i in range(n):
            out["data"].append((yield from c.rbyte(i < n - 1)))
        yield from c.stop()
    return f


@pytest.mark.parametrize("slow", (False, True), ids=("ideal", "slowest"))
@pytest.mark.parametrize("mode", MODES)
def test_target_write_and_read(mode, slow):
    b, nd = target_chip(L.slow_edges(mode) if slow else None)
    s0 = len(b.samples)
    data = [0xA5, 0x00, 0xFF, 0x5A]
    out = run_ctrl(b, write_tx(ME, [0x30] + data), mode)
    assert all(out["acks"]), out
    assert nd.peek(0x30, 4) == data
    out = run_ctrl(b, read_tx(ME, 0x30, 4), mode)
    assert all(out["acks"]) and out["data"] == data, out
    s = b.samples[s0:]
    assert events(s) == (expect_write(ME, [0x30] + data) + expect_write(ME, [0x30], stop=False)
                         + expect_read(ME, data, start="Sr"))
    # the chip's duty as a target: data set-up to the SCL rise, and no change while SCL is high
    assert not (v := timing(s, mode, check=("su_dat",))), v
    assert not b.contention


@pytest.mark.parametrize("mode", MODES)
def test_target_valid_times(mode):
    """tVD;DAT, tVD;ACK: when the target does not stretch, SDA is valid in time."""
    b, nd = target_chip()
    s0 = len(b.samples)
    run_ctrl(b, write_tx(ME, [0x30, 0x55]), mode)
    _, vd = L.data_hold_valid(b.samples[s0:], mode)
    hold_fall = Controller(Script(None), mode).hold * L.NS
    assert vd <= L.LIMITS[mode]["vd"] + hold_fall, f"valid after {vd:.0f} ns"


@pytest.mark.parametrize("mode", MODES)
def test_target_stretch_setup(mode):
    """Table 11 notes 3 and 4: a target that stretches must have its data out tr(max) + tSU;DAT before it
    releases SCL (1250 ns on a Standard-mode bus). Worst case: SDA rises as slowly as the bus allows,
    SCL rises at once."""
    edges = {SCL: L.Edges(0, 0), SDA: L.Edges(L.cyc(L.LIMITS[mode]["tr"]), 0)}
    b, nd = target_chip(edges)
    s0 = len(b.samples)
    nd.dmem(0x40, [0xFF, 0x7F, 0xBF, 0x55])
    out = run_ctrl(b, read_tx(ME, 0x40, 4), mode)
    assert out["data"] == [0xFF, 0x7F, 0xBF, 0x55], out
    assert not (v := timing(b.samples[s0:], mode, check=("su_dat",))), v


@pytest.mark.parametrize("a", [x for x in range(128) if x != ME])
def test_target_ignores_other_addresses(a):
    """Only its own address: SDA is never pulled low for another, and the next transfer still works."""
    b, nd = target_chip()
    scr_out = run_ctrl(b, write_tx(a, [0x11, 0x22]), "FM+")
    assert not any(scr_out["acks"]), f"answered address {a:#x}"
    out = run_ctrl(b, write_tx(ME, [0x05, 0x66]), "FM+")
    assert all(out["acks"]) and nd.peek(5)[0] == 0x66


@pytest.mark.parametrize("byte", (0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07) + tuple(range(8, 16)),
                         ids=lambda x: f"{x:02x}")
def test_target_reserved_addresses(byte):
    """Table 4: general call only if built with it; START byte: no device may acknowledge;
    CBUS, other formats and the Hs master codes: not for an ordinary target."""
    b, nd = target_chip()

    def f(c, out):
        yield from c.start()
        out["ack"] = yield from c.wbyte(byte)
        yield from c.stop()

    assert not run_ctrl(b, f, "FM")["ack"], f"acknowledged reserved byte {byte:#04x}"
    assert all(run_ctrl(b, write_tx(ME, [1, 2]), "FM")["acks"])


def test_target_start_byte_then_transfer():
    """3.1.15: START byte, ACK clock with nobody answering, Sr, and the real transfer."""
    b, nd = target_chip()
    s0 = len(b.samples)

    def f(c, out):
        yield from c.start()
        out["a0"] = yield from c.wbyte(0x01)
        yield from c.restart()
        out["a1"] = yield from c.wbyte(ME << 1)
        out["a2"] = yield from c.wbyte(0x07)
        out["a3"] = yield from c.wbyte(0x99)
        yield from c.stop()

    o = run_ctrl(b, f, "FM")
    assert not o["a0"] and o["a1"] and o["a2"] and o["a3"], o
    assert nd.peek(7)[0] == 0x99
    assert events(b.samples[s0:])[:4] == ["S", ("AR", 0x00), "N", "Sr"]


@pytest.mark.parametrize("n", (1, 2, 3, 5, 7))
def test_target_stop_or_start_inside_a_byte(n):
    """3.1.4: START and STOP can come at any bit: the target must drop what it had and follow."""
    b, nd = target_chip()

    def f(c, out):
        yield from c.start()
        yield from c.wbyte(ME << 1)
        for k in range(n):
            yield from c._clock(1)
        yield from c.restart()
        out["a"] = yield from c.wbyte(ME << 1)
        out["b"] = yield from c.wbyte(0x08)
        out["c"] = yield from c.wbyte(0x77)
        yield from c.stop()

    o = run_ctrl(b, f, "FM")
    assert o["a"] and o["b"] and o["c"], o
    assert nd.peek(8)[0] == 0x77


def test_target_repeated_start_after_read():
    b, nd = target_chip()
    nd.dmem(0x50, [0x12, 0x34])

    def f(c, out):
        yield from c.start()
        yield from c.wbyte(ME << 1)
        yield from c.wbyte(0x50)
        yield from c.restart()
        yield from c.wbyte(ME << 1 | 1)
        out["x"] = yield from c.rbyte(False)
        yield from c.restart()
        yield from c.wbyte(ME << 1 | 1)
        out["y"] = yield from c.rbyte(False)
        yield from c.stop()

    o = run_ctrl(b, f, "FM")
    assert o["x"] == 0x12 and o["y"] == 0x34, o


def test_target_ignores_hs_traffic():
    """5.3.5: F/S devices stay out of an Hs transfer: master code, Sr, fast bytes, STOP."""
    b, nd = target_chip()
    s0 = len(b.samples)

    def f(c, out):
        yield from c.start()
        out["mc"] = yield from c.wbyte(0x08)
        c.tlow, c.thigh, c.hold = 8, 8, 2
        yield from c.restart()
        out["a"] = yield from c.wbyte(ME << 1)      # same bits as our address, but in Hs mode
        yield from c.stop()

    run_ctrl(b, f, "FM")
    assert all(run_ctrl(b, write_tx(ME, [1, 2]), "FM")["acks"]), "the target did not recover"


@pytest.mark.parametrize("A", (0x000, 0x0FF, 0x155, 0x2AA, 0x3FF))
def test_target_10bit(A):
    b, nd = target_chip(addr10=A)

    def f(c, out):
        hi = 0xF0 | ((A >> 7) & 6)
        yield from c.start()
        out["a"] = [(yield from c.wbyte(hi)), (yield from c.wbyte(A & 0xFF)), (yield from c.wbyte(0x60)),
                    (yield from c.wbyte(0xAB))]
        yield from c.restart()
        out["b"] = yield from c.wbyte(hi | 1)
        out["d"] = yield from c.rbyte(False)
        yield from c.stop()

    o = run_ctrl(b, f, "FM")
    assert all(o["a"]) and o["b"], o
    assert nd.peek(0x60)[0] == 0xAB


def test_target_general_call():
    """3.1.13: built with it, the target acknowledges address 0 with W."""
    b, nd = target_chip(general_call=True)
    assert all(run_ctrl(b, write_tx(0, [0x12, 0x34]), "FM")["acks"])
    assert nd.peek(F.I2C_GC_MEM, 2) == [0x12, 0x34]


def spikes(pin, width, seed, every=(300, 900)):
    """Noise at the chip's inputs: `pin` is flipped for `width` cycles, at random times."""
    r = random.Random(seed)
    state = [400, 0]

    def f(t):
        if t >= state[0]:
            state[1], state[0] = width, t + r.randint(*every)
        if state[1]:
            state[1] -= 1
            return 1 << pin
        return 0
    return f


@pytest.mark.parametrize("width", (1, 2, 3))
@pytest.mark.parametrize("line", ("scl", "sda"))
@pytest.mark.parametrize("mode", ("FM", "FM+"))
def test_target_spikes(mode, line, width):
    """Table 10, tSP: a Fast or Fast-mode Plus device suppresses spikes under 50 ns on SCL and SDA.
    A 49 ns spike can be caught by 3 clock edges at 48 MHz."""
    b, nd = target_chip()
    b.noise = spikes(SCL if line == "scl" else SDA, width, width)
    out = run_ctrl(b, write_tx(ME, [0x10, 0xA5, 0x5A, 0xC3]), mode)
    assert all(out["acks"]), f"NACK with {width}-cycle spikes on {line}: {out}"
    assert nd.peek(0x10, 3) == [0xA5, 0x5A, 0xC3], nd.peek(0x10, 3)


@pytest.mark.parametrize("width", (1, 2, 3))
@pytest.mark.parametrize("line", ("scl", "sda"))
@pytest.mark.parametrize("mode", ("FM", "FM+"))
def test_master_spikes(mode, line, width):
    t = Target()
    t.mem[0x10:0x12] = [0x11, 0x22]
    b, nd = master(mode=mode, target=t)
    b.noise = spikes(SCL if line == "scl" else SDA, width, width + 10)
    st, got = request(b, nd, 0x50, [0x10], 2)
    assert st == 0 and got == [0x11, 0x22], (st, got)


def test_spikes_do_break_an_unfiltered_input():
    """Negative control: the noise used above is enough to break the bus when the filter is off."""
    b, nd = target_chip(filt=0)
    b.noise = spikes(SCL, 2, 2)
    out = run_ctrl(b, write_tx(ME, [0x10, 0xA5, 0x5A, 0xC3]), "FM")
    assert not all(out["acks"]) or nd.peek(0x10, 3) != [0xA5, 0x5A, 0xC3]
