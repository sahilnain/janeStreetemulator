# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_rxu: unit test for src/unit/pe_rxu.v (RX unit).

What is checked
---------------
`plan()` computes, from docs/spec/blocks.md Section 4 and the random input
waveform, the cycle of every sample and its value, the done cycle D and the
drive-mode writes. Every cycle the testbench compares done, push, push_data,
busy and pw_mask/pw_val.
  SRC 0: window i at W_i = S + FIRST + i*B; sample j at W_i + (j+1)*STEP;
         majority of K <= j < N-K (ties 0); D = W_7 + N*STEP + 1
  SRC 1: bit i = pin in cycle T_i + SDLY (T_i = i-th tx_stick after S)
  SRC 2: bit i = pin SDLY cycles after the i-th selected EPIN edge
  drive: bit 0 in S; bit i at W_i / i-th tx_dtick / i-th opposite edge
1. own_timer_random: 60 packets, random B/N/K/STEP/FIRST, random waveform.
2. tick_random: SRC 1 with random tick spacing and SDLY 0..15.
3. ext_edge_random: SRC 2, rising and falling, SDLY 0..15.
4. drive_modes: OEN with SRC 0/1/2, LSB and MSB.
5. hold_rxrdy: HOLD mode: RXRDY set by after_end (same cycle as done, or
   later), DATA read clears it, overrun when a byte ends while RXRDY is
   set, clr_ovr, restart clears; STAT bits.
6. stop_restart: stop in the middle: no done/push, idle next cycle.
7. rx_nbits_all: 1-16 bits, both orders, right-aligned in DATA.
8. rx_inv: INV inverts the samples.
9. rx_hunt_chain: sync hunt, gap-free words, stuffing removed across words.
10. rx_hunt_glitch: a single 0 in idle is not a sync.
11. rx_stuff_error: a 1 where a stuffed 0 belongs: sticky error, cleared by start.
12. rx_dpll_tracks: a sender 4% off, one sample per bit.
13. rx_se0_no_sample: DIFF: no sample in SE0.
"""

import random

import cocotb
import pe_isa as I
from tbutil import fall, param, settle, start, v

NP = param("NUM_PINS", 8)
INS = ("cfg_we", "cfg_idx", "cfg_data", "rd_idx", "rd_strobe", "start", "start_data", "stop",
       "restart", "pin_in", "tx_stick", "tx_dtick", "after_end", "ferr", "clr_ovr")


async def cfg(dut, idx, data):
    dut.cfg_we.value, dut.cfg_idx.value, dut.cfg_data.value = 1, idx, data
    await fall(dut)
    dut.cfg_we.value = 0


async def init(dut):
    for s in INS:
        getattr(dut, s).value = 0
    await start(dut)


def bit(data, k, msb):
    return (data >> (7 - k if msb else k)) & 1


def assemble(bits, msb):
    return sum(b << (7 - i if msb else i) for i, b in enumerate(bits))


def waveform(T, pins, hold=6):
    """Random per-cycle input words; each pin keeps a level for 1..hold cycles."""
    w, cur, left = [], [0] * NP, [0] * NP
    for _ in range(T):
        for p in pins:
            if left[p] == 0:
                cur[p] = random.getrandbits(1)
                left[p] = random.randint(1, hold)
            left[p] -= 1
        w.append(sum(cur[p] << p for p in range(NP)))
    return w


def plan(c, S, wave, ticks, dticks):
    """-> (sample cycles, bits, D, writes {cycle: bit index})"""
    ip, ep = c["ipin"], c["epin"]
    pin = lambda t, p: (wave[t] >> p) & 1  # noqa: E731
    writes = {S: 0} if c["oen"] else {}
    if c["src"] == 0:
        B = c["B"] or 1
        W = [S + c["FIRST"] + i * B for i in range(8)]
        st = c["STEP"] or 1
        N = c["N"] or 1
        bits, D = [], None
        for i in range(8):
            samples = [pin(W[i] + (j + 1) * st, ip) for j in range(N)]
            cnt = [samples[j] for j in range(N) if c["K"] <= j < N - c["K"]]
            votes = max(0, N - 2 * c["K"])
            bits.append(int(2 * sum(cnt) > votes))
        D = W[7] + N * st + 1
        for i in range(1, 8):
            if c["oen"]:
                writes[W[i]] = i
        return None, bits, D, writes
    if c["src"] == 1:
        evs = [t for t in ticks if t > S][:8]
        drv = [t for t in dticks if t > S][:7]
    else:
        sel = 1 - c["eedge"]
        evs, drv = [], []
        for t in range(S + 1, len(wave)):
            if pin(t, ep) != pin(t - 1, ep):
                (evs if pin(t, ep) == sel else drv).append(t)
        evs, drv = evs[:8], drv[:7]
    smp = [t + c["SDLY"] for t in evs]
    bits = [pin(t, ip) for t in smp]
    if c["oen"]:
        for i, t in enumerate(drv):
            writes[t + c["ODLY"]] = i + 1   # only written while busy (checked in run)
    return smp, bits, smp[7], writes


async def setup(dut, c):
    await cfg(dut, I.RX_PINS, I.rx_pins(c["ipin"], c["opin"], c["oen"]))
    await cfg(dut, I.RX_BIT, c["B"])
    await cfg(dut, I.RX_FRAME, I.rx_frame(c["msb"], c["hold"]))
    await cfg(dut, I.RX_VOTE, I.rx_vote(c["N"], c["K"]))
    await cfg(dut, I.RX_STEP, c["STEP"])
    await cfg(dut, I.RX_FIRST, c["FIRST"])
    await cfg(dut, I.RX_TICK, I.rx_tick(c["src"], c["epin"], c["eedge"], c["SDLY"]))
    await cfg(dut, I.RX_ODLY, c.get("ODLY", 0))


def rand_cfg(src, oen=0):
    ip, op, ep = random.sample(range(NP), 3)
    c = dict(ipin=ip, opin=op, epin=ep, oen=oen, src=src, msb=random.getrandbits(1), hold=0,
             eedge=random.getrandbits(1), SDLY=random.randint(0, 15), K=0, N=1, STEP=1, B=1, FIRST=0)
    if src == 0:
        N = random.randint(1, 7)
        K = random.randint(0, (N - 1) // 2)
        STEP = random.randint(1, 5)
        c.update(N=N, K=K, STEP=STEP, B=N * STEP + 1 + random.randint(0, 6),
                 FIRST=random.choice([0, 1, 2, random.randint(3, 30)]))
    return c


async def run(dut, c, data=0, T=None):
    """One packet with a random waveform; checks every cycle."""
    T = T or (c["FIRST"] + 9 * max(c["B"], 1) + 30 if c["src"] == 0 else 400)
    wave = waveform(T + 40, [c["ipin"], c["epin"]], hold=6 if c["src"] != 2 else 4)
    S = 3
    ticks, dticks, t = [], [], S + 1
    while t < T:
        ticks.append(t)
        dticks.append(t + random.randint(0, 1))
        t += random.randint(1, 6)
    dticks = sorted(set(dticks))
    smp, bits, D, writes = plan(c, S, wave, ticks, dticks)
    byte = assemble(bits, c["msb"])
    for t in range(D + 3):
        dut.pin_in.value = wave[t]
        dut.tx_stick.value = int(t in ticks)
        dut.tx_dtick.value = int(t in dticks)
        dut.start.value, dut.start_data.value = int(t == S), data
        await settle()
        assert v(dut.done) == int(t == D), f"done at {t}, D={D} cfg={c}"
        assert v(dut.push) == int(t == D and not c["hold"])
        if t == D:
            assert v(dut.push_data) == byte, f"byte {v(dut.push_data):02x} != {byte:02x} cfg={c}"
        assert v(dut.busy) == int(S < t <= D), f"busy at {t}"
        if c["oen"]:
            om = 1 << c["opin"]
            if t in writes and t <= D:
                assert v(dut.pw_mask) == om, f"drive at {t} cfg={c}"
                assert (v(dut.pw_val) >> c["opin"]) & 1 == bit(data, writes[t], c["msb"]), f"bit {writes[t]}"
            else:
                assert v(dut.pw_mask) == 0, f"no drive at {t} cfg={c}"
        else:
            assert v(dut.pw_mask) == 0
        await fall(dut)
    dut.tx_stick.value = dut.tx_dtick.value = 0
    return byte


@cocotb.test(timeout_time=100, timeout_unit="ms")
async def own_timer_random(dut):
    await init(dut)
    for _ in range(60):
        c = rand_cfg(0)
        await setup(dut, c)
        await run(dut, c)


@cocotb.test(timeout_time=100, timeout_unit="ms")
async def tick_random(dut):
    await init(dut)
    for _ in range(40):
        c = rand_cfg(1)
        await setup(dut, c)
        await run(dut, c)


@cocotb.test(timeout_time=100, timeout_unit="ms")
async def ext_edge_random(dut):
    await init(dut)
    for _ in range(40):
        c = rand_cfg(2)
        await setup(dut, c)
        await run(dut, c)


@cocotb.test(timeout_time=100, timeout_unit="ms")
async def drive_modes(dut):
    await init(dut)
    for src in (0, 1, 2):
        for msb in (0, 1):
            for _ in range(4):
                c = rand_cfg(src, oen=1)
                c["msb"] = msb
                if src == 2:
                    c["SDLY"] = 0
                c["ODLY"] = random.choice([0, 1, 3]) if src else 0
                await setup(dut, c)
                await run(dut, c, data=random.getrandbits(8))


async def packet_fixed(dut, data_bits):
    """SRC 1 packet: 8 ticks one cycle apart, pin = the given bits."""
    dut.start.value = 1
    await fall(dut)
    dut.start.value = 0
    for b in data_bits:
        dut.pin_in.value = b << 2
        dut.tx_stick.value = 1
        await settle()
        d = v(dut.done)
        await fall(dut)
    dut.tx_stick.value = 0
    assert d == 1


async def after_end(dut, data_rd=False):
    dut.after_end.value = 1
    if data_rd:
        dut.rd_idx.value, dut.rd_strobe.value = 0, 1
    await fall(dut)
    dut.after_end.value = 0
    dut.rd_strobe.value = 0


@cocotb.test()
async def hold_rxrdy(dut):
    await init(dut)
    c = rand_cfg(1)
    c.update(ipin=2, epin=5, opin=6, hold=1, SDLY=0, msb=0)
    await setup(dut, c)
    dut.rd_idx.value = 1
    # byte 1, after_end in the done cycle -> DATA = byte, RXRDY
    dut.start.value = 1
    await fall(dut)
    dut.start.value = 0
    for i, b in enumerate([1, 0, 1, 0, 0, 1, 0, 1]):
        dut.pin_in.value = b << 2
        dut.tx_stick.value = 1
        dut.after_end.value = int(i == 7)
        await settle()
        assert v(dut.push) == 0, "HOLD: no push"
        await fall(dut)
    dut.tx_stick.value = dut.after_end.value = 0
    await settle()
    assert v(dut.rxrdy) == 1 and v(dut.overrun) == 0
    dut.rd_idx.value = 0
    await settle()
    assert v(dut.rd_data) == 0xA5
    dut.rd_idx.value = 1
    await settle()
    assert v(dut.rd_data) & 0xF == 0b0010            # rxrdy
    dut.ferr.value = 1
    await settle()
    assert v(dut.rd_data) & 0xF == 0b0110            # + ferr (from ERX)
    dut.ferr.value = 0
    # byte 2 while RXRDY still set, after_end later -> overrun, DATA = new byte
    await packet_fixed(dut, [0, 1, 1, 1, 1, 0, 0, 0])
    await fall(dut, 3)
    assert v(dut.rxrdy) == 1 and v(dut.overrun) == 0, "RXRDY only at after_end"
    await after_end(dut)
    await settle()
    assert v(dut.overrun) == 1 and v(dut.rxrdy) == 1
    dut.rd_idx.value = 0
    await settle()
    assert v(dut.rd_data) == 0x1E
    # read clears RXRDY; clr_ovr clears overrun
    dut.rd_strobe.value = 1
    await fall(dut)
    dut.rd_strobe.value = 0
    await settle()
    assert v(dut.rxrdy) == 0
    dut.clr_ovr.value = 1
    await fall(dut)
    dut.clr_ovr.value = 0
    await settle()
    assert v(dut.overrun) == 0
    # byte 3: after_end together with a DATA read of the old byte: no overrun
    await packet_fixed(dut, [1] * 8)
    await after_end(dut)
    await packet_fixed(dut, [0] * 8)
    await after_end(dut, data_rd=True)
    await settle()
    assert v(dut.rxrdy) == 1 and v(dut.overrun) == 0
    dut.rd_idx.value = 0
    await settle()
    assert v(dut.rd_data) == 0x00
    # restart clears RXRDY and overrun
    await packet_fixed(dut, [1] * 8)
    await after_end(dut)
    await settle()
    assert v(dut.overrun) == 1
    dut.restart.value = 1
    await fall(dut)
    dut.restart.value = 0
    await settle()
    assert v(dut.rxrdy) == 0 and v(dut.overrun) == 0


@cocotb.test()
async def stop_restart(dut):
    await init(dut)
    for src in (0, 1, 2):
        for stop_sig in ("stop", "restart"):
            c = rand_cfg(src)
            await setup(dut, c)
            dut.start.value = 1
            await fall(dut)
            dut.start.value = 0
            for t in range(random.randint(1, 5)):
                dut.tx_stick.value = 1
                dut.pin_in.value = (t & 1) << c["epin"]
                await fall(dut)
            getattr(dut, stop_sig).value = 1
            await fall(dut)
            getattr(dut, stop_sig).value = 0
            for t in range(300):
                dut.tx_stick.value = t & 1
                dut.pin_in.value = ((t >> 1) & 1) << c["epin"]
                await settle()
                assert v(dut.busy) == 0 and v(dut.done) == 0 and v(dut.push) == 0
                await fall(dut)
            dut.tx_stick.value = 0


async def jump(dut, n):
    from cocotb.triggers import ClockCycles
    if n > 0:
        await ClockCycles(dut.clk, n, rising=False)
    await settle()


@cocotb.test(timeout_time=400, timeout_unit="ms")
async def extremes(dut):
    """Max 16-bit B / FIRST / STEP, N = 15, out-of-range pins, unused indices."""
    await init(dut)
    # own timer: max B / FIRST / STEP (N = 1), then max N / K with a short step
    for N, K, STEP, B, FIRST in ((1, 0, 0xFFFE, 0xFFFF, 0xFFFF), (15, 7, 2, 31, 3)):
        c = dict(ipin=NP - 1, opin=0, epin=1, oen=1, src=0, msb=0, hold=0, eedge=0, SDLY=15,
                 K=K, N=N, STEP=STEP, B=B, FIRST=FIRST)
        await setup(dut, c)
        dut.pin_in.value = 1 << (NP - 1)          # all ones
        dut.start.value, dut.start_data.value = 1, 0xC3
        await fall(dut)
        dut.start.value = 0
        D = FIRST + 7 * B + N * STEP + 1
        await jump(dut, D - 2)
        assert v(dut.done) == 0
        await jump(dut, 1)
        assert v(dut.done) == 1 and v(dut.push_data) == 0xFF, (N, K, STEP, B)
        await fall(dut)
        assert v(dut.busy) == 0
    # pins >= NUM_PINS read 0 and drive nothing
    if NP < 16:
        c.update(ipin=15, opin=14, epin=13, src=1, SDLY=0, N=1, K=0, STEP=1, B=1, FIRST=0)
        await setup(dut, c)
        dut.pin_in.value = (1 << NP) - 1
        dut.start.value = 1
        await settle()
        assert v(dut.pw_mask) == 0
        await fall(dut)
        dut.start.value = 0
        for _ in range(8):
            dut.tx_stick.value = 1
            await settle()
            d = v(dut.done)
            await fall(dut)
        dut.tx_stick.value = 0
        assert d == 1
    # unused status indices
    for idx in range(2, 8):
        dut.rd_idx.value = idx
        await settle()
        assert v(dut.rd_data) == 0
    for idx in range(8):
        for b in range(16):
            await cfg(dut, idx, 1 << b)
        await cfg(dut, idx, 0)


# ---------------------------------------------------------------- NBITS, line decoding
async def rx_setup(dut, B=4, frame=0, line=0, src=I.SRC_DPLL, pins=None, first=0):
    await cfg(dut, I.RX_PINS, I.rx_pins(1) if pins is None else pins)
    await cfg(dut, I.RX_BIT, B)
    await cfg(dut, I.RX_VOTE, I.rx_vote(1))
    await cfg(dut, I.RX_STEP, 1)
    await cfg(dut, I.RX_FIRST, first)
    await cfg(dut, I.RX_TICK, I.rx_tick(src))
    await cfg(dut, I.RX_FRAME, frame)
    await cfg(dut, I.RX_ODLY, line)


async def feed(dut, wave, pin=1, pin2=None, wave2=None, hold=0):
    """Starts the unit, plays one level per cycle; returns words from push / done."""
    dut.start.value = 1
    got = []
    for c, lv in enumerate(wave):
        val = lv << pin
        if pin2 is not None:
            val |= wave2[c] << pin2
        dut.pin_in.value = val
        await settle()
        if v(dut.done):
            got.append(v(dut.push_data))
        await fall(dut)
        dut.start.value = 0
    return got


def nrzi_wave(bits, B, start=1):
    out, lv = [], start
    for b in bits:
        lv = lv if b else 1 - lv
        out += [lv] * B
    return out


def stuff(bits, n=6, ones=0):
    out = []
    for b in bits:
        out.append(b)
        ones = ones + 1 if b else 0
        if ones == n:
            out.append(0)
            ones = 0
    return out


def byte_bits(data):
    return [(d >> i) & 1 for d in data for i in range(8)]


@cocotb.test()
async def test_rx_nbits_all(dut):
    """1-16 bits, both orders, right-aligned, on the tick source; DATA has all 16 bits."""
    await init(dut)
    for field, n in [(k, k) for k in range(1, 17)] + [(0, 8), (20, 16)]:
        for msb in (0, 1):
            await rx_setup(dut, frame=I.rx_frame(msb, hold=1, nbits=field), src=I.SRC_TXTICK)
            word = random.getrandbits(n)
            dut.start.value = 1
            await fall(dut)
            dut.start.value = 0
            for i in range(n):
                b = (word >> (n - 1 - i if msb else i)) & 1
                dut.pin_in.value = b << 1
                dut.tx_stick.value = 1
                await settle()
                assert v(dut.done) == int(i == n - 1), (n, i)
                await fall(dut)
                dut.tx_stick.value = 0
            dut.after_end.value = 1
            await fall(dut)
            dut.after_end.value = 0
            dut.rd_idx.value = 0
            await settle()
            assert v(dut.rd_data) == word, (n, msb, hex(word), hex(v(dut.rd_data)))
            await fall(dut, 2)
            assert not v(dut.busy)


@cocotb.test()
async def test_rx_own_timer_16(dut):
    """16 windows on the own timer: the window count reaches 16 and bit 15 lands in DATA."""
    await init(dut)
    B = 4
    for msb in (0, 1):
        await rx_setup(dut, B=B, frame=I.rx_frame(msb, hold=1, nbits=16), src=I.SRC_OWN, first=2)
        word = random.getrandbits(16) | 0x8001
        wave = []
        for i in range(16):
            wave += [(word >> (15 - i if msb else i)) & 1] * B
        got = await feed(dut, wave + [0] * B)
        assert len(got) == 1, got
        dut.after_end.value = 1
        await fall(dut)
        dut.after_end.value = 0
        dut.rd_idx.value = 0
        await settle()
        assert v(dut.rd_data) == word, (msb, hex(word), hex(v(dut.rd_data)))
        await fall(dut, 2)
        assert not v(dut.busy)


@cocotb.test()
async def test_rx_inv(dut):
    await init(dut)
    await rx_setup(dut, B=4, frame=I.rx_frame(inv=1), first=2)
    wave = [(1 - ((0x3A >> i) & 1)) for i in range(8) for _ in range(4)]
    assert await feed(dut, wave + [0] * 8) == [0x3A]
    await fall(dut, 2)


@cocotb.test()
async def test_rx_hunt_chain(dut):
    """Idle, sync, then bytes: hunting ignores idle 1s; bytes chain with no gap; stuffing
    is removed across byte boundaries; the sync's last 1 counts."""
    await init(dut)
    for B in (4, 32):
        for data in ([0xFF, 0xFF, 0x12], [0x1F, 0x80, 0xFE], [0x00, 0x55, 0xAA, 0xFF]):
            await rx_setup(dut, B, I.rx_frame(chain=1), I.rx_line(nrzi=1, hunt=1, stuffn=6))
            bits = [1] * 3 + [0] * 7 + stuff([1] + byte_bits(data))
            wave = nrzi_wave(bits, B)
            got = await feed(dut, [1] * 5 + wave + [wave[-1]] * (B // 2))
            assert got == data, (B, data, got)
            assert v(dut.busy), "chain: still busy"
            dut.stop.value = 1
            await fall(dut)
            dut.stop.value = 0
            await fall(dut)
            assert not v(dut.busy)


@cocotb.test()
async def test_rx_hunt_glitch(dut):
    """One 0 in idle is not a sync: a 1 needs two 0s before it."""
    await init(dut)
    B = 8
    await rx_setup(dut, B, I.rx_frame(chain=1), I.rx_line(nrzi=1, hunt=1))
    bits = [1, 1, 0, 1, 1, 0, 1] + [0, 0, 1] + byte_bits([0xC5])
    wave = nrzi_wave(bits, B)
    assert await feed(dut, wave + [wave[-1]] * 4) == [0xC5]
    dut.stop.value = 1
    await fall(dut)
    dut.stop.value = 0
    await fall(dut)


@cocotb.test()
async def test_rx_stuff_error(dut):
    await init(dut)
    B = 4
    await rx_setup(dut, B, I.rx_frame(chain=1), I.rx_line(nrzi=1, hunt=1, stuffn=6))
    bits = [0, 0, 1] + [1] * 7 + [0]              # 5 data 1s, the error, one 1, one 0: 7 bits
    wave = nrzi_wave(bits, B)
    await feed(dut, [1] * 3 + wave)
    dut.rd_idx.value = 1
    await settle()
    st = v(dut.rd_data)
    assert st & I.RX_ST_SERR, hex(st)
    assert (st & I.RX_ST_NBITS) >> 8 == 7, hex(st)
    dut.stop.value = 1
    await fall(dut)
    dut.stop.value = 0
    await fall(dut, 2)
    await rx_setup(dut, B, I.rx_frame(chain=1), I.rx_line(nrzi=1, hunt=1, stuffn=6))
    dut.start.value = 1
    await fall(dut)
    dut.start.value = 0
    await settle()
    assert not v(dut.rd_data) & I.RX_ST_SERR, "cleared by start"
    dut.stop.value = 1
    await fall(dut)
    dut.stop.value = 0
    await fall(dut)


@cocotb.test()
async def test_rx_dpll_tracks(dut):
    """Sender 4% slow or fast at B = 4 (5 and 3 cycle bits mixed): one sample per bit."""
    await init(dut)
    for bit in (4.1667, 3.88, 33.33 / 8) * 40:     # a 7-bit run of exactly 30 cycles is rare
        await rx_setup(dut, 4, I.rx_frame(chain=1), I.rx_line(nrzi=1, hunt=1, stuffn=6))
        data = [random.getrandbits(8) for _ in range(8)] + [0xFF, 0xFF]
        bits = [0] * 7 + stuff([1] + byte_bits(data))
        wave, lv, acc = [], 1, 0.0
        for b in bits:
            lv = lv if b else 1 - lv
            acc += bit
            n = int(acc)
            acc -= n
            wave += [lv] * n
        assert await feed(dut, [1, 1] + wave + [lv] * 2) == data, bit
        dut.stop.value = 1
        await fall(dut)
        dut.stop.value = 0
        await fall(dut)


@cocotb.test()
async def test_rx_se0_no_sample(dut):
    """With DIFF no sample is taken while both pins are low. The sample would be due
    in the SE0, one cycle before a detector can stop the unit."""
    await init(dut)
    B = 4
    await rx_setup(dut, B, 0, 0, pins=I.rx_pins(1, i2pin=2, diff=1), first=2)
    dut.start.value = 1
    dut.pin_in.value = 0
    await fall(dut)
    dut.start.value = 0
    taken = 0
    for c in range(40):
        await settle()
        taken += v(dut.bit_v)
        await fall(dut)
    assert taken == 0, taken
    dut.pin_in.value = 0b100
    for c in range(8):
        await settle()
        taken += v(dut.bit_v)
        await fall(dut)
    assert taken >= 1
    dut.stop.value = 1
    await fall(dut)
    dut.stop.value = 0
    await fall(dut)
