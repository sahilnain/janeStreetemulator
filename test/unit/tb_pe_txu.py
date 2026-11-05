# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_txu: unit test for src/unit/pe_txu.v (TX unit).

What is checked
---------------
`expect()` builds, from docs/spec/blocks.md Section 3.3, the outputs of every
cycle of a packet started in cycle S (offset c = cycle - S):
  c = 0:            data bit 0; SCK idle (CPHA 0) or active (CPHA 1)
  c = k*B, k=1..7:  data bit k, dtick; SCK idle (CPHA 0) / active (CPHA 1)
  c = k*B + H:      stick; SCK active (CPHA 0) / idle (CPHA 1)   (H = B//2 >= 1)
  c = 8*B:          done; SCK idle (CPHA 0 only)
  busy = 1 for c = 1 .. 8*B
The testbench compares pw_mask, pw_val, stick, dtick, done and busy in every
cycle from S to S + 8B + 3.
1. own_timer_sweep: B in 1..12, 33, 100, 0 (-> 1); CPOL x CPHA; LSB/MSB;
   data and/or clock pin; random distinct pins; random data.
2. back_to_back: the next start in the cycle after done (gap-free bytes).
3. stop_abort: stop at a random point: idle next cycle, no done, no writes.
4. ext_clock: SPI slave mode; random edges on EPIN (rising / falling
   selected); bit i written in the cycle edge i is seen, done on edge 8;
   opposite edges ignored.
5. stretch: I2C SCL with STRETCH; the pin is read back through a 2-cycle
   delay and a slave holds it low for X cycles: each SCK high phase is
   (B - H) + 2 + X cycles; without stretch it is B - H.
6. status_and_cfg: STAT busy bit; config readback via den/dpin.
7. ext_clock_lead: LEAD = 1 (SPI slave CPHA 1): no write in S, bit i on
   selected edge i+1, opposite edges only give done after bit 7.
8. lead_ignored_on_own_timer: LEAD has no effect with EXT = 0.
9. nbits_all: 1-16 bits (field 0 -> 8, >= 16 -> 16), both orders, B = 1, 3, 8.
10. inv: INV inverts the data bits.
11. line_random: NRZI and stuffing against an independent model.
12. chain_gapless: words chain on the bit grid; stuffing carries across; CRC word.
13. eop_diff: DIFF pins, SE0 for 2B, J for B, stuff bit before the EOP.
14. eop_stop: stop inside the EOP leaves no state behind.
15. crc_stream: the CRC stream has data bits only.
"""

import random

import cocotb
import pe_isa as I
from tbutil import fall, param, settle, start, v

NP = param("NUM_PINS", 8)


async def cfg(dut, idx, data):
    dut.cfg_we.value, dut.cfg_idx.value, dut.cfg_data.value = 1, idx, data
    await fall(dut)
    dut.cfg_we.value = 0


async def init(dut):
    for s in ("cfg_we", "cfg_idx", "cfg_data", "rd_idx", "start", "start_data", "stop", "pin_in", "next_ok", "next_crc", "next_data"):
        getattr(dut, s).value = 0
    await start(dut)


async def setup(dut, B, dpin, den, cpin, cen, msb, cpol, cpha, stretch=0, ext=0, epin=0, eedge=0, lead=0):
    await cfg(dut, I.TX_PINS, I.tx_pins(dpin, den, cpin, cen))
    await cfg(dut, I.TX_BIT, B)
    await cfg(dut, I.TX_FRAME, msb)
    await cfg(dut, I.TX_CLK, I.tx_clk(cpol, cpha, stretch))
    await cfg(dut, I.TX_TICK, I.tx_tick(ext, epin, eedge, lead))


def bit(data, k, msb):
    return (data >> (7 - k if msb else k)) & 1


def expect(B, data, dpin, den, cpin, cen, msb, cpol, cpha):
    """{offset: (mask, val, stick, dtick, done)}"""
    Be = B or 1
    H = Be // 2
    ev = {}

    def w(c, pin, lv, **f):
        m, val, st, dt, dn = ev.get(c, (0, 0, 0, 0, 0))
        if pin is not None:
            m |= 1 << pin
            val = (val & ~(1 << pin)) | (lv << pin)
        ev[c] = (m, val, st | f.get("st", 0), dt | f.get("dt", 0), dn | f.get("dn", 0))

    act, idle = 1 - cpol, cpol
    if den:
        w(0, dpin, bit(data, 0, msb))
    if cen:
        w(0, cpin, act if cpha else idle)
    for c in range(1, 8 * Be + 1):
        if c % Be == 0:
            k = c // Be
            if k == 8:
                w(c, None, 0, dn=1)
                if cen and not cpha:
                    w(c, cpin, idle)
            else:
                w(c, None, 0, dt=1)
                if den:
                    w(c, dpin, bit(data, k, msb))
                if cen:
                    w(c, cpin, act if cpha else idle)
        elif H and c % Be == H:
            w(c, None, 0, st=1)
            if cen:
                w(c, cpin, idle if cpha else act)
    return ev


async def run_packet(dut, B, data, pins, msb, cpol, cpha, next_start=None):
    dpin, den, cpin, cen = pins
    Be = B or 1
    ev = expect(B, data, dpin, den, cpin, cen, msb, cpol, cpha)
    dut.start.value, dut.start_data.value = 1, data
    for c in range(8 * Be + 3):
        await settle()
        m, val, st, dt, dn = ev.get(c, (0, 0, 0, 0, 0))
        got = (v(dut.pw_mask), v(dut.pw_val) & v(dut.pw_mask), v(dut.stick), v(dut.dtick), v(dut.done))
        assert got == (m, val, st, dt, dn), f"B={B} c={c} cfg={pins, msb, cpol, cpha}: {got} != {(m, val, st, dt, dn)}"
        assert v(dut.busy) == int(1 <= c <= 8 * Be), f"busy at c={c}"
        await fall(dut)
        dut.start.value = 0
        if next_start is not None and c + 1 == 8 * Be + 1:
            return  # caller starts the next packet in this cycle


def rand_pins():
    dpin, cpin = random.sample(range(NP), 2)
    den, cen = random.choice([(1, 0), (1, 1), (0, 1)])
    return dpin, den, cpin, cen


@cocotb.test(timeout_time=50, timeout_unit="ms")
async def own_timer_sweep(dut):
    await init(dut)
    for B in list(range(1, 13)) + [33, 100, 0]:
        for cpol in (0, 1):
            for cpha in (0, 1):
                pins = rand_pins()
                if B in (0, 1):
                    pins = (pins[0], 1, pins[2], 0)     # no clock below B = 2
                msb = random.getrandbits(1)
                await setup(dut, B, *pins, msb, cpol, cpha)
                await run_packet(dut, B, random.getrandbits(8), pins, msb, cpol, cpha)


@cocotb.test()
async def back_to_back(dut):
    await init(dut)
    B, pins = 3, (2, 1, 5, 1)
    await setup(dut, B, *pins, 0, 0, 0)
    for _ in range(3):
        await run_packet(dut, B, random.getrandbits(8), pins, 0, 0, 0, next_start=True)
    await run_packet(dut, B, 0xA5, pins, 0, 0, 0)


@cocotb.test()
async def stop_abort(dut):
    await init(dut)
    for _ in range(20):
        B = random.randint(2, 9)
        await setup(dut, B, 1, 1, 2, 1, 0, 0, 0)
        dut.start.value, dut.start_data.value = 1, 0x5A
        await fall(dut)
        dut.start.value = 0
        await fall(dut, random.randint(0, 8 * B - 2))
        assert v(dut.busy) == 1
        dut.stop.value = 1
        await fall(dut)
        dut.stop.value = 0
        for _ in range(8 * B + 2):
            await settle()
            assert v(dut.busy) == 0 and v(dut.done) == 0 and v(dut.pw_mask) == 0
            assert v(dut.stick) == 0 and v(dut.dtick) == 0
            await fall(dut)


@cocotb.test(timeout_time=20, timeout_unit="ms")
async def ext_clock(dut):
    await init(dut)
    for eedge in (0, 1):
        for msb in (0, 1):
            dpin, epin = 3, 6
            await setup(dut, 5, dpin, 1, 0, 0, msb, 0, 0, ext=1, epin=epin, eedge=eedge)
            lvl = 0
            dut.pin_in.value = 0
            await fall(dut, 2)
            data = random.getrandbits(8)
            dut.start.value, dut.start_data.value = 1, data
            await settle()
            assert v(dut.pw_mask) == 1 << dpin and (v(dut.pw_val) >> dpin) & 1 == bit(data, 0, msb)
            await fall(dut)
            dut.start.value = 0
            k = 0
            while True:
                for _ in range(random.randint(1, 4)):        # quiet cycles
                    await settle()
                    assert v(dut.pw_mask) == 0 and v(dut.done) == 0
                    await fall(dut)
                lvl ^= 1
                dut.pin_in.value = lvl << epin
                await settle()
                sel = (lvl == 1) if eedge == 0 else (lvl == 0)
                if not sel:
                    assert v(dut.pw_mask) == 0 and v(dut.done) == 0
                elif k == 7:
                    assert v(dut.done) == 1 and v(dut.pw_mask) == 0
                    await fall(dut)
                    assert v(dut.busy) == 0
                    break
                else:
                    k += 1
                    assert v(dut.pw_mask) == 1 << dpin, f"edge {k}"
                    assert (v(dut.pw_val) >> dpin) & 1 == bit(data, k, msb)
                await fall(dut)


@cocotb.test(timeout_time=20, timeout_unit="ms")
async def stretch(dut):
    await init(dut)
    B, H, cpin, dpin = 10, 5, 4, 2
    for st_en, extra in ((0, 0), (1, 0), (1, 7)):
        await setup(dut, B, dpin, 1, cpin, 1, 0, 0, 0, stretch=st_en)
        # pad = SCK register (written in t, on the pad from t+1); the slave holds
        # the wire low for `extra` cycles after the 2nd rise; pin_in = wire 2 cycles late
        pad, hold_until, wire = 0, -1, {}
        rises, falls, sticks = [], [], []
        dut.start.value, dut.start_data.value = 1, 0x3C
        for t in range(400):
            wire[t] = int(pad and t > hold_until)
            dut.pin_in.value = wire.get(t - 2, 0) << cpin
            await settle()
            if v(dut.stick):
                sticks.append(t)
            new = pad
            if v(dut.pw_mask) >> cpin & 1:
                new = (v(dut.pw_val) >> cpin) & 1
                if new and not pad:
                    rises.append(t)
                    if len(rises) == 2:
                        hold_until = t + extra
                if not new and pad:
                    falls.append(t)
            pad = new
            done = v(dut.done)
            await fall(dut)
            dut.start.value = 0
            if done:
                break
        assert len(rises) == 8 and len(falls) == 8, (rises, falls)
        # the sample tick: at the release write without STRETCH; with STRETCH in
        # the first cycle pin_in shows SCK high (>= w + 3, after any slave hold)
        if not st_en:
            assert sticks == rises, (sticks, rises)
        else:
            exp = [r + 3 + (extra if i == 1 else 0) for i, r in enumerate(rises)]
            assert sticks == exp, f"sticks {sticks} != {exp}"
        for i, (r, f) in enumerate(zip(rises, falls)):
            exp = (B - H) + (2 + (extra if i == 1 else 0) if st_en else 0)
            assert f - r == exp, f"stretch={st_en} extra={extra} clock {i}: high {f - r} != {exp}"


@cocotb.test()
async def status_and_cfg(dut):
    await init(dut)
    await setup(dut, 4, 7, 1, 1, 0, 0, 0, 0)
    await settle()
    assert v(dut.den) == 1 and v(dut.dpin) == 7
    dut.rd_idx.value = 0
    await settle()
    assert v(dut.rd_data) & 1 == 0
    dut.start.value = 1
    await fall(dut)
    dut.start.value = 0
    await settle()
    assert v(dut.rd_data) & 1 == 1
    dut.rd_idx.value = 3
    await settle()
    assert v(dut.rd_data) == 0


async def at_cycle(dut, n):
    """Advance n cycles quickly (no per-cycle Python), then settle."""
    from cocotb.triggers import ClockCycles
    if n > 0:
        await ClockCycles(dut.clk, n, rising=False)
    await settle()


@cocotb.test(timeout_time=200, timeout_unit="ms")
async def extremes(dut):
    """Max 16-bit B (every timer bit toggles), out-of-range pins, unused cfg/status indices."""
    await init(dut)
    for B in (0xFFFF, 0x8001):
        await setup(dut, B, 1, 1, 2, 1, 1, 1, 0)
        dut.start.value, dut.start_data.value = 1, 0x81
        await settle()
        assert v(dut.pw_mask) == 0b110
        await fall(dut)
        dut.start.value = 0
        c = 1
        for k in (1, 7):                         # boundaries k*B, and the mid tick of bit 7
            await at_cycle(dut, k * B - c)
            c = k * B
            assert v(dut.dtick) == 1 and v(dut.pw_mask) == 0b110, f"B={B} k={k}"
            assert (v(dut.pw_val) >> 1) & 1 == bit(0x81, k, 1)
        await at_cycle(dut, B // 2)
        c += B // 2
        assert v(dut.stick) == 1
        await at_cycle(dut, 8 * B - c)
        assert v(dut.done) == 1 and v(dut.busy) == 1
        await fall(dut)
        assert v(dut.busy) == 0
    # pins >= NUM_PINS: no write at all (mask 0), ext pin reads 0
    if NP < 16:
        await setup(dut, 2, 15, 1, 14, 1, 0, 0, 0)
        dut.start.value = 1
        await settle()
        assert v(dut.pw_mask) == 0
        await fall(dut)
        dut.start.value = 0
        await fall(dut, 20)
    # unused cfg indices are ignored; unused status indices read 0
    await setup(dut, 3, 0, 1, 0, 0, 0, 0, 0)
    for idx in (5, 6, 7):
        await cfg(dut, idx, 0xFFFF)
    await settle()
    assert v(dut.dpin) == 0 and v(dut.den) == 1
    for idx in range(1, 8):
        dut.rd_idx.value = idx
        await settle()
        assert v(dut.rd_data) == 0
    # the epin / cfg bits all toggle: walk every config bit once
    for idx in range(5):
        for b in range(16):
            await cfg(dut, idx, 1 << b)
        await cfg(dut, idx, 0)
    dut.pin_in.value = (1 << NP) - 1
    await fall(dut, 2)
    dut.pin_in.value = 0
    await fall(dut, 2)


@cocotb.test(timeout_time=20, timeout_unit="ms")
async def ext_clock_lead(dut):
    await init(dut)
    for eedge in (0, 1):
        for msb in (0, 1):
            dpin, epin = 3, 6
            await setup(dut, 5, dpin, 1, 0, 0, msb, 0, 0, ext=1, epin=epin, eedge=eedge, lead=1)
            lvl = eedge                       # idle level: the first toggle is the selected edge
            dut.pin_in.value = lvl << epin
            await fall(dut, 2)
            data = random.getrandbits(8)
            dut.start.value, dut.start_data.value = 1, data
            await settle()
            assert v(dut.pw_mask) == 0, "LEAD: nothing written at start"
            await fall(dut)
            dut.start.value = 0
            k = 0                             # bits written
            while True:
                for _ in range(random.randint(1, 4)):
                    await settle()
                    assert v(dut.pw_mask) == 0 and v(dut.done) == 0
                    await fall(dut)
                lvl ^= 1
                dut.pin_in.value = lvl << epin
                await settle()
                selected = lvl == (1 - eedge)
                if selected:
                    assert k < 8
                    assert v(dut.pw_mask) == 1 << dpin, f"selected edge {k}"
                    assert (v(dut.pw_val) >> dpin) & 1 == bit(data, k, msb), f"bit {k}"
                    assert v(dut.done) == 0
                    k += 1
                else:
                    assert v(dut.pw_mask) == 0
                    if k == 8 and random.getrandbits(1):
                        # a stray second selected edge after bit 7: nothing changes
                        dut.pin_in.value = (lvl ^ 1) << epin
                        await fall(dut)
                        await settle()
                        assert v(dut.pw_mask) == 0 and v(dut.busy) == 1
                        dut.pin_in.value = lvl << epin
                        await settle()
                    assert v(dut.done) == int(k == 8), f"after {k} bits"
                    if k == 8:
                        await fall(dut)
                        assert v(dut.busy) == 0
                        break
                await fall(dut)


@cocotb.test()
async def lead_ignored_on_own_timer(dut):
    """Found by formal: LEAD left set while the unit uses its own timer must not
    change the bits (it only applies to the external clock)."""
    await init(dut)
    pins = (2, 1, 5, 0)
    await setup(dut, 3, *pins, 0, 0, 0, ext=0, epin=1, eedge=0, lead=1)
    await run_packet(dut, 3, 0xA5, pins, 0, 0, 0)


# ---------------------------------------------------------------- NBITS, line coding
async def line_setup(dut, B, frame=0, line=0, dpin=1):
    await cfg(dut, I.TX_PINS, I.tx_pins(dpin))
    await cfg(dut, I.TX_BIT, B)
    await cfg(dut, I.TX_CLK, 0)
    await cfg(dut, I.TX_TICK, 0)
    await cfg(dut, I.TX_FRAME, frame)
    await cfg(dut, I.TX_LINE, line)


async def capture(dut, data, limit=2000, next_words=()):
    """Start a packet. Returns (writes as (cycle, mask, val), done cycle, chain cycles).
    next_words: words offered for CHAIN, as (data, is_crc)."""
    q = list(next_words)
    dut.next_ok.value = int(bool(q))
    dut.next_crc.value = q[0][1] if q else 0
    dut.next_data.value = q[0][0] if q else 0
    dut.start.value, dut.start_data.value = 1, data
    wr, ch = [], []
    for c in range(limit):
        await settle()
        if v(dut.pw_mask):
            wr.append((c, v(dut.pw_mask), v(dut.pw_val)))
        if v(dut.done):
            await fall(dut)
            return wr, c, ch
        if v(dut.chain):
            ch.append(c)
            q.pop(0)
        await fall(dut)
        dut.start.value = 0
        dut.next_ok.value = int(bool(q))
        dut.next_crc.value = q[0][1] if q else 0
        dut.next_data.value = q[0][0] if q else 0
    raise AssertionError("no done")


@cocotb.test()
async def test_nbits_all(dut):
    await init(dut)
    for field, n in [(k, k) for k in range(1, 17)] + [(0, 8), (17, 16), (31, 16)]:
        for msb in (0, 1):
            for B in (1, 3, 8):
                data = random.getrandbits(16)
                await line_setup(dut, B, I.tx_frame(msb, nbits=field))
                wr, d, _ = await capture(dut, data)
                assert d == n * B, f"n={n} B={B}: done at {d}"
                assert [w[0] for w in wr] == [i * B for i in range(n)], (n, B, wr)
                bits = [(w[2] >> 1) & 1 for w in wr]
                exp = [(data >> (n - 1 - i if msb else i)) & 1 for i in range(n)]
                assert bits == exp, (n, msb, bits, exp)
                await fall(dut, 2)
                assert not v(dut.busy)


@cocotb.test()
async def test_inv(dut):
    await init(dut)
    await line_setup(dut, 2, I.tx_frame(inv=1))
    wr, _, _ = await capture(dut, 0x35)
    assert [(w[2] >> 1) & 1 for w in wr] == [((0x35 >> i) & 1) ^ 1 for i in range(8)]


def line_model(words, stuffn, nrzi):
    """Levels on the wire for raw words (bit lists): stuffing after stuffn ones, NRZI."""
    out, lvl, ones = [], 1, 0
    for b in [x for w in words for x in w]:
        lvl = (lvl if b else 1 - lvl) if nrzi else b
        out.append(lvl)
        ones = ones + 1 if b else 0
        if stuffn and ones == stuffn:
            lvl = (1 - lvl) if nrzi else 0
            out.append(lvl)
            ones = 0
    return out


def levels(wr, B, pin=1):
    """One level per bit; every write must be on the bit grid."""
    assert all(w[0] % B == 0 for w in wr), wr
    return [(w[2] >> pin) & 1 for w in wr]


@cocotb.test()
async def test_line_random(dut):
    await init(dut)
    for _ in range(60):
        B = random.choice((1, 2, 4, 7))
        nrzi = random.getrandbits(1)
        stuffn = random.choice((0, 1, 2, 6))
        n = random.randint(1, 16)
        data = random.choice((0xFFFF, 0, random.getrandbits(16)))
        await line_setup(dut, B, I.tx_frame(nbits=n), I.tx_line(nrzi=nrzi, stuffn=stuffn))
        wr, d, _ = await capture(dut, data)
        exp = line_model([[(data >> i) & 1 for i in range(n)]], stuffn, nrzi)
        assert levels(wr, B) == exp, (B, nrzi, stuffn, n, hex(data), levels(wr, B), exp)
        assert d == len(exp) * B, (d, len(exp), B)
        await fall(dut, 2)


@cocotb.test()
async def test_chain_gapless(dut):
    """Words chain on the bit grid; stuffing and NRZI carry across the boundary; no done
    until the last; the CRC word is 16 bits MSB-first."""
    await init(dut)
    B = 4
    await line_setup(dut, B, I.tx_frame(chain=1), I.tx_line(nrzi=1, stuffn=6))
    words = [0xFF, 0x7F, 0x03]
    crc = 0xBEEF
    wr, d, ch = await capture(dut, words[0], next_words=[(words[1], 0), (words[2], 0), (crc, 1)])
    raw = [[(w >> i) & 1 for i in range(8)] for w in words] + [[(crc >> (15 - i)) & 1 for i in range(16)]]
    exp = line_model(raw, 6, 1)
    assert levels(wr, B) == exp
    assert d == len(exp) * B
    assert len(ch) == 3
    await fall(dut, 2)
    # with nothing offered it is one word
    wr, d, ch = await capture(dut, 0x55)
    assert d == 8 * B and not ch
    await fall(dut, 2)


@cocotb.test()
async def test_eop_diff(dut):
    await init(dut)
    for B in (4, 32):
        for last in (0x00, 0xFC):              # 0xFC ends in six 1s: a stuff bit first
            await line_setup(dut, B, 0, I.tx_line(nrzi=1, eop=1, diff=1, d2pin=3, stuffn=6))
            wr, d, _ = await capture(dut, last)
            nb = len(line_model([[(last >> i) & 1 for i in range(8)]], 6, 1))
            assert d == (nb + 3) * B, (d, nb, B)
            for c, m, val in wr[:nb]:
                assert m == 0b1010 and ((val >> 1) & 1) != ((val >> 3) & 1), "not differential"
            assert wr[nb][0] == nb * B and wr[nb][1:] == (0b1010, 0), "SE0"
            assert wr[nb + 1][0] == (nb + 2) * B and wr[nb + 1][1:] == (0b1010, 0b0010), "J"
            assert len(wr) == nb + 2
            assert not v(dut.bit_v)
            assert v(dut.d2_mask) == 0b1000
            await fall(dut, 2)
            assert not v(dut.busy)


@cocotb.test()
async def test_eop_stop(dut):
    await init(dut)
    await line_setup(dut, 4, 0, I.tx_line(eop=1, diff=1, d2pin=3))
    dut.start.value, dut.start_data.value = 1, 0
    await fall(dut)
    dut.start.value = 0
    await fall(dut, 8 * 4 + 2)             # inside SE0
    dut.stop.value = 1
    await fall(dut)
    dut.stop.value = 0
    await fall(dut)
    assert not v(dut.busy)
    await line_setup(dut, 2, I.tx_frame(nbits=2), 0)
    wr, d, _ = await capture(dut, 3)       # a stopped EOP leaves no state behind
    assert d == 4 and len(wr) == 2


@cocotb.test()
async def test_crc_stream(dut):
    """bit_v / bit_o give the data bits only: not stuffed bits, not the CRC word."""
    await init(dut)
    B = 2
    await line_setup(dut, B, I.tx_frame(chain=1), I.tx_line(nrzi=1, stuffn=6))
    dut.next_ok.value, dut.next_crc.value, dut.next_data.value = 1, 1, 0xAAAA
    dut.start.value, dut.start_data.value = 1, 0xFF
    seen = []
    for _ in range(200):
        await settle()
        if v(dut.bit_v):
            seen.append(v(dut.bit_o))
        if v(dut.chain):
            dut.next_ok.value = 0
        if v(dut.done):
            break
        await fall(dut)
        dut.start.value = 0
    assert seen == [1] * 8, seen
