# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_core: unit test for src/core/pe_core.v, run on the harness
test/unit/hdl/pe_core_tb.v (core + real imem, FIFOs and pin fabric).

What is checked
---------------
Small programs built with pe_isa are loaded into instruction memory and
run. Each test checks cycle-exact behaviour on the pins, FIFOs and status.
NS = number of slots = max(2, NUM_CTX), so one instruction takes NS cycles.

 1. issue_rate        set/set/jmp loop: high for NS cycles, low for 2*NS.
 2. delay_timing      delay field 0..7 and `delay n` give exactly
                      NS*ceil((d+1)/NS) cycles.
 3. djnz_loops        mov + djnz on ctr0 and ctr1 give exactly N pulses.
 4. pull_out_order    pull + out 4 + out 4: nibble order for LSB-first and
                      MSB-first.
 5. in_push_order     in 4 + in 4 + push: byte assembly for both shift
                      directions.
 6. wait_and_timeout  wait completes when the pin matches; wait with a
                      timeout sets TO and jmp TO is taken.
 7. jmp_conditions    ALWAYS, TXF_EMPTY (both ways), OSR_EMPTY (both ways).
 8. vote_sets_c       vote on a pin held at 1/0 sets C, and jmp C follows it.
                      A context without a vote unit treats vote as nop.
 9. push_block_nb     blocking push stalls at a full RX FIFO; push nb drops
                      the byte and sets overrun.
10. owner_violation   a write to a pin owned by another context is
                      ignored and sets owner_err.
11. restart           restart jumps to start_pc; the stale fetched
                      instruction is not executed.
12. two_contexts      (NUM_CTX >= 2) two contexts toggle two pins with
                      different delays; both periods are exact and
                      independent, and halting one does not disturb the other.
13. uart_loopback     (NUM_CTX == 2) UART TX firmware on ctx0, vote-based
                      RX firmware on ctx1, pin 0 looped back to pin 1:
                      24 random bytes arrive intact, and the TX waveform is
                      checked edge by edge against the bit grid.
"""

import random

import cocotb
import pe_fw
import pe_isa as I
import uart
from cocotb.triggers import FallingEdge
from tbutil import fall, param, start, v


class H:
    """Harness helper."""

    def __init__(self, dut):
        self.dut = dut
        self.nc = param("NUM_CTX", 2)
        self.np = param("NUM_PINS", 8)
        self.ns = max(2, self.nc)
        self.aw = (param("IMEM_DEPTH", 64) - 1).bit_length()
        self.ctx_w = max(1, (self.nc - 1).bit_length())
        self.nw, self.sw = param("VOTE_N_W", 4), param("VOTE_S_W", 12)
        mask = param("VOTE_CTX_MASK", 3)
        self.vote_ctx = [c for c in range(self.nc) if (mask >> c) & 1]
        self.no_vote_ctx = [c for c in range(self.nc) if not (mask >> c) & 1]
        self.ext = 0        # value driven on pad_in for non-looped pins
        self.loop = []      # list of (src_pin, dst_pin)
        self.samples = []   # pad_out per cycle
        self.oes = []

    async def init(self):
        d = self.dut
        for s in ("im_we", "im_waddr", "im_wdata", "txf_wr", "txf_wdata", "rxf_rd", "rxf_clr",
                  "ctx_run", "ctx_restart", "ctx_start_pc", "ctx_shift_msb", "pin_owner",
                  "pin_od", "owner_clr", "pad_in"):
            getattr(d, s).value = 0
        d.vote_n.value = self.pack([9] * self.nc, self.nw)
        d.vote_k.value = self.pack([1] * self.nc, self.nw)
        d.vote_s.value = self.pack([12] * self.nc, self.sw)
        await start(d)
        cocotb.start_soon(self._pads())

    @staticmethod
    def pack(vals, w):
        return sum(x << (i * w) for i, x in enumerate(vals))

    async def _pads(self):
        d = self.dut
        while True:
            await FallingEdge(d.clk)
            po = v(d.pad_out)
            self.samples.append(po)
            self.oes.append(v(d.pad_oe))
            x = self.ext
            for s, t in self.loop:
                x = (x & ~(1 << t)) | (((po >> s) & 1) << t)
            d.pad_in.value = x

    async def load(self, prog, base=0):
        d = self.dut
        for i, w in enumerate(prog):
            d.im_we.value, d.im_waddr.value, d.im_wdata.value = 1, base + i, w
            await fall(d)
        d.im_we.value = 0

    def owners(self, owner_list):
        self.dut.pin_owner.value = self.pack(owner_list, self.ctx_w)

    async def run(self, ctxs, start_pcs=None, msb=0):
        """Restart the given contexts at their start PCs, then let them run."""
        d = self.dut
        spc = start_pcs or {}
        d.ctx_start_pc.value = self.pack([spc.get(c, 0) for c in range(self.nc)], self.aw)
        d.ctx_shift_msb.value = sum(((msb >> c) & 1) << c for c in range(self.nc)) if msb else 0
        d.ctx_restart.value = sum(1 << c for c in ctxs)
        await fall(d)
        d.ctx_restart.value = 0
        d.ctx_run.value = sum(1 << c for c in ctxs)

    async def halt(self):
        self.dut.ctx_run.value = 0
        await fall(self.dut)

    def pc(self, c):
        return (v(self.dut.ctx_pc) >> (c * self.aw)) & ((1 << self.aw) - 1)

    def pin_trace(self, pin, start=0):
        return [(s >> pin) & 1 for s in self.samples[start:]]

    async def host_push(self, byte):
        d = self.dut
        while v(d.txf_full):
            await fall(d)
        d.txf_wr.value, d.txf_wdata.value = 1, byte
        await fall(d)
        d.txf_wr.value = 0

    async def host_pop(self):
        d = self.dut
        while v(d.rxf_empty):
            await fall(d)
        x = v(d.rxf_rdata)
        d.rxf_rd.value = 1
        await fall(d)
        d.rxf_rd.value = 0
        return x


def runs(bits):
    """Run lengths of a 0/1 list. Drops the first (partial) run, then starts at
    the first high run, so the result does not depend on where the trace began."""
    out, cur, n = [], bits[0], 0
    for b in bits:
        if b == cur:
            n += 1
        else:
            out.append((cur, n))
            cur, n = b, 1
    out = out[1:]
    while out and out[0][0] == 0:
        out = out[1:]
    return out


def cost(ns, d):
    return ns * -(-(d + 1) // ns)


async def setup(dut):
    h = H(dut)
    await h.init()
    h.owners([0] * h.np)
    return h


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def issue_rate(dut):
    h = await setup(dut)
    await h.load([I.set_(0, I.SRC_1), I.set_(0, I.SRC_0), I.jmp(0)])
    t0 = len(h.samples)
    await h.run([0])
    await fall(dut, 30 * h.ns)
    r = runs(h.pin_trace(0, t0 + 4))
    assert r[:6] == [(1, h.ns), (0, 2 * h.ns)] * 3, r[:6]


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def delay_timing(dut):
    h = await setup(dut)
    for d in range(8):
        await h.halt()
        await h.load([I.set_(0, I.SRC_1, dly=d), I.set_(0, I.SRC_0, dly=d), I.jmp(0)])
        t0 = len(h.samples)
        await h.run([0])
        await fall(dut, 12 * cost(h.ns, d) + 10)
        r = runs(h.pin_trace(0, t0 + 4))
        c = cost(h.ns, d)
        assert r[:4] == [(1, c), (0, c + h.ns)] * 2, f"dly={d}: {r[:4]}"
    for n in (0, 1, 5, 30, 101):
        await h.halt()
        await h.load([I.set_(0, I.SRC_1), I.delay(n), I.set_(0, I.SRC_0), I.jmp(0)])
        t0 = len(h.samples)
        await h.run([0])
        await fall(dut, 6 * (cost(h.ns, n) + 3 * h.ns) + 10)
        r = runs(h.pin_trace(0, t0 + 4))
        assert r[:2] == [(1, h.ns + cost(h.ns, n)), (0, 2 * h.ns)], f"delay {n}: {r[:2]}"


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def djnz_loops(dut):
    h = await setup(dut)
    for ctr, n in ((0, 5), (1, 3), (0, 1)):
        await h.halt()
        await h.load([I.set_(0, I.SRC_0), I.mov(ctr, n), I.set_(0, I.SRC_1), I.set_(0, I.SRC_0),
                      I.djnz(ctr, 2), I.jmp(5)])
        t0 = len(h.samples)
        await h.run([0])
        await fall(dut, (4 * n + 10) * h.ns)
        tr = h.pin_trace(0, t0)
        pulses = sum(1 for a, b in zip(tr, tr[1:]) if a == 0 and b == 1)
        assert pulses == n, f"ctr{ctr} n={n}: {pulses} pulses"
        assert h.pc(0) == 5


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def pull_out_order(dut):
    h = await setup(dut)
    for msb in (0, 1):
        await h.halt()
        prog = [I.pull(), I.out(0, 4), I.out(0, 4), I.jmp(3)]
        await h.load(prog)
        await h.host_push(0xA5)
        t0 = len(h.samples)
        await h.run([0], msb=msb)
        await fall(dut, 10 * h.ns)
        seen = []
        for s in h.samples[t0:]:
            if not seen or seen[-1] != s & 0xF:
                seen.append(s & 0xF)
        exp = [0x5, 0xA] if msb == 0 else [0xA, 0x5]
        assert seen[-2:] == exp, f"msb={msb}: {seen}"


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def in_push_order(dut):
    h = await setup(dut)
    h.ext = 0xC3
    await fall(dut, 3)
    for msb, exp in ((0, 0xC3), (1, 0x3C)):
        await h.halt()
        await h.load([I.in_(0, 4), I.in_(4, 4), I.push(), I.jmp(3)])
        await h.run([0], msb=msb)
        assert await h.host_pop() == exp, f"msb={msb}"


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def wait_and_timeout(dut):
    h = await setup(dut)
    prog = [I.set_(0, I.SRC_0), I.set_(1, I.SRC_0), I.mov(0, 10), I.wait(3, 1, to_ctr=0),
            I.jmp(7, I.TO), I.set_(0, I.SRC_1), I.jmp(6), I.set_(1, I.SRC_1), I.jmp(8)]
    await h.load(prog)
    # pin 3 never goes high -> timeout path (pin 1)
    h.ext = 0
    await h.run([0])
    await fall(dut, 30 * h.ns)
    assert (h.samples[-1] & 3) == 0b10 and v(dut.ctx_to) & 1
    # pin 3 high -> normal path (pin 0), TO cleared
    await h.halt()
    h.ext = 0b1000
    await fall(dut, 3)
    await h.run([0])
    await fall(dut, 30 * h.ns)
    assert (h.samples[-1] & 3) == 0b01 and not (v(dut.ctx_to) & 1)
    # plain wait (no timeout) holds the PC until the pin matches
    await h.halt()
    h.ext = 0
    await h.load([I.wait(3, 1), I.set_(0, I.SRC_1), I.jmp(2)])
    await h.run([0])
    await fall(dut, 20 * h.ns)
    assert h.pc(0) == 0
    h.ext = 0b1000
    await fall(dut, 10 * h.ns)
    assert h.pc(0) == 2


async def cond_taken(h, cond, prep=()):
    """Run prep + jmp cond -> pin1, else pin0. Returns 1 if taken."""
    await h.halt()
    base = list(prep)
    t = len(base) + 5
    prog = base + [I.set_(0, I.SRC_0), I.set_(1, I.SRC_0), I.jmp(t, cond), I.set_(0, I.SRC_1),
                   I.jmp(t - 1), I.set_(1, I.SRC_1), I.jmp(t + 1)]
    await h.load(prog)
    await h.run([0])
    await fall(h.dut, (len(prog) + 6) * h.ns)
    pins = h.samples[-1] & 3
    assert pins in (1, 2), pins
    return int(pins == 2)


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def jmp_conditions(dut):
    h = await setup(dut)
    assert await cond_taken(h, I.ALWAYS) == 1
    assert await cond_taken(h, I.TXF_EMPTY) == 1
    await h.host_push(0x11)
    assert await cond_taken(h, I.TXF_EMPTY) == 0
    assert await cond_taken(h, I.OSR_EMPTY, [I.pull()]) == 0        # 8 bits left
    await h.host_push(0x22)
    assert await cond_taken(h, I.OSR_EMPTY, [I.pull(), I.out(7, 8)]) == 1


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def vote_sets_c(dut):
    h = await setup(dut)
    c = h.vote_ctx[0]
    h.owners([c] * h.np)
    dut.vote_n.value = h.pack([3] * h.nc, h.nw)
    dut.vote_k.value = h.pack([0] * h.nc, h.nw)
    dut.vote_s.value = h.pack([2] * h.nc, h.sw)
    prog = [I.set_(0, I.SRC_0), I.set_(1, I.SRC_0), I.vote(2), I.jmp(6, I.C), I.set_(0, I.SRC_1),
            I.jmp(5), I.set_(1, I.SRC_1), I.jmp(7)]
    await h.load(prog)
    for lvl, exp in ((1, 0b10), (0, 0b01)):
        await h.halt()
        h.ext = lvl << 2
        await fall(dut, 3)
        await h.run([c])
        await fall(dut, 20 * h.ns)
        assert (h.samples[-1] & 3) == exp, f"level {lvl}"
        assert ((v(dut.ctx_c) >> c) & 1) == lvl
    if h.no_vote_ctx:
        c = h.no_vote_ctx[0]
        h.owners([c] * h.np)
        await h.halt()
        h.ext = 1 << 2
        await h.run([c])
        await fall(dut, 20 * h.ns)
        assert (h.samples[-1] & 3) == 0b01, "vote must be a nop without a vote unit"


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def push_block_nb(dut):
    h = await setup(dut)
    depth = 4
    await h.load([I.push(), I.jmp(0)])
    await h.run([0])
    await fall(dut, 40 * h.ns)
    assert v(dut.rxf_full) == 1 and v(dut.rxf_overrun) == 0
    assert h.pc(0) == 0, "blocking push must hold the PC"
    for _ in range(depth):
        await h.host_pop()
    await h.halt()
    await h.load([I.push(1), I.jmp(0)])
    await h.run([0])
    await fall(dut, 40 * h.ns)
    assert v(dut.rxf_full) == 1 and v(dut.rxf_overrun) == 1
    dut.rxf_clr.value = 1
    await fall(dut)
    dut.rxf_clr.value = 0


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def owner_violation(dut):
    h = await setup(dut)
    if h.nc < 2:
        return
    h.owners([1] + [0] * (h.np - 1))
    await h.load([I.set_(0, I.SRC_0), I.set_(1, I.SRC_0), I.jmp(2)])
    await h.run([0])
    await fall(dut, 10 * h.ns)
    s = h.samples[-1]
    assert s & 1 == 1, "pin 0 belongs to ctx1 and must not change"
    assert s & 2 == 0, "pin 1 belongs to ctx0 and must change"
    assert v(dut.owner_err) == 1
    dut.owner_clr.value = 1
    await fall(dut)
    dut.owner_clr.value = 0


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def restart(dut):
    h = await setup(dut)
    await h.load([I.set_(0, I.SRC_1), I.set_(0, I.SRC_0), I.jmp(0)])
    await h.load([I.set_(2, I.SRC_1), I.jmp(11)], base=10)
    await h.run([0])
    await fall(dut, 10 * h.ns)
    # restart while running, without halting first
    dut.ctx_start_pc.value = 10
    dut.ctx_restart.value = 1
    await fall(dut)
    dut.ctx_restart.value = 0
    t0 = len(h.samples)
    await fall(dut, 10 * h.ns)
    assert h.pc(0) == 11
    tr0 = h.pin_trace(0, t0 + 2)
    assert len(set(tr0)) == 1, "old program kept running after restart"
    assert h.samples[-1] & 4


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def two_contexts(dut):
    h = await setup(dut)
    if h.nc < 2:
        return
    h.owners([0, 1] + [0] * (h.np - 2))
    await h.load([I.set_(0, I.SRC_1, dly=7), I.set_(0, I.SRC_0, dly=7), I.jmp(0)])
    await h.load([I.set_(1, I.SRC_1), I.delay(20), I.set_(1, I.SRC_0), I.delay(20), I.jmp(20)], base=20)
    t0 = len(h.samples)
    await h.run([0, 1], {0: 0, 1: 20})
    await fall(dut, 300)
    c7 = cost(h.ns, 7)
    r0 = runs(h.pin_trace(0, t0 + 4))
    assert r0[:4] == [(1, c7), (0, c7 + h.ns)] * 2, r0[:4]
    hi1 = h.ns + cost(h.ns, 20)
    r1 = runs(h.pin_trace(1, t0 + 4))
    assert r1[:4] == [(1, hi1), (0, hi1 + h.ns)] * 2, r1[:4]
    # halt ctx1 only; ctx0 keeps exactly the same timing
    dut.ctx_run.value = 1
    t1 = len(h.samples)
    await fall(dut, 200)
    r0 = runs(h.pin_trace(0, t1))
    assert r0[:4] == [(1, c7), (0, c7 + h.ns)] * 2, r0[:4]


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def uart_loopback(dut):
    h = await setup(dut)
    if h.nc != 2 or 1 not in h.vote_ctx:
        return
    N, K, S = 9, 1, 12
    B = pe_fw.rx_bit_period(N, S)
    h.owners([0, 1] + [0] * (h.np - 2))
    h.loop = [(0, 1)]
    dut.vote_n.value = h.pack([N] * 2, h.nw)
    dut.vote_k.value = h.pack([K] * 2, h.nw)
    dut.vote_s.value = h.pack([S] * 2, h.sw)
    await h.load(pe_fw.uart_tx(0, B, base=0))
    await h.load(pe_fw.uart_rx(1, N, S, base=20), base=20)
    t0 = len(h.samples)
    await h.run([0, 1], {0: 0, 1: 20})
    data = [random.getrandbits(8) for _ in range(24)]

    async def feeder():
        for x in data:
            await h.host_push(x)

    cocotb.start_soon(feeder())
    got = []
    for _ in data:
        got.append(await h.host_pop())
    assert got == data, f"sent {data}\ngot  {got}"
    await fall(dut, 2 * B)  # let the last stop bit finish before decoding the TX line
    assert uart.decode(h.pin_trace(0, t0), B) == data
