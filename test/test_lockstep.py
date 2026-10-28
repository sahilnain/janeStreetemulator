# SPDX-License-Identifier: Apache-2.0
"""
test_lockstep: RTL vs. Python reference model, compared every clock cycle.

What this file does
-------------------
Runs tt_um_sahil_protoemu (through tb.v) and the cycle-accurate model
model/pe_model.Chip side by side. On every FALLING edge it:
  1. compares the chip outputs: uo_out, uio_out, uio_oe (bits the model
     marks unknown, i.e. an RTL X from reading an empty RX FIFO, are skipped);
  2. compares internal state (RTL only, skipped in gate-level sim): every
     context's PC, counters, OSR/ISR, flags and stall counter, every vote
     unit, the slot counter, FIFO counts, pin registers, RUN, IMEM_ADDR,
     error flags, host-interface address/phase;
  3. reads the inputs the RTL will sample at the next rising edge
     (ui_in, uio_in, rst_n) and steps the model with them.
All stimulus is applied on RISING edges (half a cycle away from sampling),
so model and RTL always see the same inputs. Any difference fails the test
with the cycle number, the signal, the RTL value and the model value.

Stimulus
--------
* Phase A (protocol): the host-bus driver loads a RANDOM 64-word program
  (all opcodes, including reserved ones), random pin owners / open-drain /
  vote settings / start PCs / shift directions, starts the contexts, then
  performs random host operations (FIFO traffic, status reads, RUN/RESTART
  toggles, error clears, IRQ enables, IMEM commits while running...).
  The uio pins get random external values with partial loopback.
* Phase B (raw): ui_in is completely random every cycle (the host-bus
  protocol is violated on purpose), so the host interface and register file
  are compared under arbitrary input.

Tests: three seeds, plus a deterministic UART loopback run in lockstep.
"""

import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "common"))

import cocotb  # noqa: E402
import pe_fw  # noqa: E402
import pe_host as R  # noqa: E402
import pe_isa as I  # noqa: E402
from cocotb.clock import Clock  # noqa: E402
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge  # noqa: E402
from pe_model import Chip, Config  # noqa: E402
from pe_model.blocks import STATE  # noqa: E402


def res(sig):
    """int value of an RTL signal, or None if it holds X/Z."""
    val = sig.value
    return int(val) if val.is_resolvable else None


class Lockstep:
    def __init__(self, dut, cfg=None):
        self.dut = dut
        self.cfg = cfg or Config()
        self.chip = Chip(self.cfg)
        self.armed = False
        self.cycles = 0
        self.compared = 0
        self.last_line = 0xFF
        self.internal = self._internal_handles()

    def _internal_handles(self):
        up = self.dut.user_project
        if not hasattr(up, "u_core"):  # gate-level netlist: pins only
            return []
        core, c = up.u_core, self.cfg
        h = []
        for g in range(c.num_ctx):
            ctx = core.g_ctx[g].u_ctx
            for name in STATE + ("stall",):
                h.append((f"ctx{g}.{name}", getattr(ctx, name),
                          lambda m, g=g, name=name: getattr(m.core.ctxs[g], name)))
            if (c.vote_ctx_mask >> g) & 1:
                vu = core.g_ctx[g].g_vote.u_vote
                for rtl, mod in (("running", "running"), ("done", "done"), ("bit_out", "bit"),
                                 ("idx", "idx"), ("ones", "ones"), ("scnt", "scnt"), ("sel", "sel")):
                    h.append((f"vote{g}.{rtl}", getattr(vu, rtl),
                              lambda m, g=g, mod=mod: getattr(m.core.votes[g], mod)))
        h += [
            ("core.slot", core.slot, lambda m: m.core.slot),
            ("txf.cnt", up.u_txf.cnt, lambda m: m.txf.cnt),
            ("rxf.cnt", up.u_rxf.cnt, lambda m: m.rxf.cnt),
            ("txf.overrun", up.u_txf.overrun, lambda m: m.txf.overrun),
            ("rxf.overrun", up.u_rxf.overrun, lambda m: m.rxf.overrun),
            ("pins.out_r", up.u_pins.out_r, lambda m: m.pins.out_r),
            ("pins.oe_r", up.u_pins.oe_r, lambda m: m.pins.oe_r),
            ("pins.owner_err", up.u_pins.owner_err, lambda m: m.pins.owner_err),
            ("regs.run", up.u_regs.ctx_run, lambda m: m.regs.run),
            ("regs.imaddr", up.u_regs.imaddr, lambda m: m.regs.imaddr),
            ("regs.imem_err", up.u_regs.imem_err, lambda m: m.regs.imem_err),
            ("regs.pin_owner", up.u_regs.pin_owner, lambda m: m.regs.owner),
            ("regs.pin_od", up.u_regs.pin_od, lambda m: m.regs.od),
            ("host.r_addr", up.u_host_if.r_addr, lambda m: m.host.addr),
            ("host.wphase", up.u_host_if.wphase, lambda m: m.host.wphase),
            ("host.rphase", up.u_host_if.rphase, lambda m: m.host.rphase),
        ]
        return h

    def _fail(self, name, rtl, mod):
        raise AssertionError(f"cycle {self.cycles}: {name} RTL={rtl!r} model={mod!r}")

    def compare(self):
        d, o = self.dut, self.chip.outputs()
        known = o["uo_known"]
        rtl_bits = str(d.uo_out.value)  # MSB first, may contain X
        for i in range(8):
            if (known >> i) & 1:
                rb = rtl_bits[7 - i]
                if rb not in "01" or int(rb) != (o["uo_out"] >> i) & 1:
                    self._fail(f"uo_out[{i}]", d.uo_out.value, f"{o['uo_out']:08b} (known mask {known:08b})")
        for name, key in (("uio_out", "uio_out"), ("uio_oe", "uio_oe")):
            r = res(getattr(d, name))
            if r != o[key]:
                self._fail(name, getattr(d, name).value, f"{o[key]:08b}")
        for name, sig, get in self.internal:
            r, m = res(sig), get(self.chip)
            if r != m:
                self._fail(name, sig.value, m)
        self.compared += 1

    async def run(self):
        d = self.dut
        while True:
            await FallingEdge(d.clk)
            if self.armed:
                self.compare()
            out, oe = res(d.uio_out), res(d.uio_oe)
            if out is not None and oe is not None:
                self.last_line = (out & oe) | (0xFF & ~oe)
            self.chip.step(int(d.ui_in.value), int(d.uio_in.value), int(d.rst_n.value))
            self.cycles += 1


async def start(dut, ls):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0xFF
    dut.rst_n.value = 0
    await RisingEdge(dut.clk)
    cocotb.start_soon(ls.run())
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 3)
    ls.armed = True  # both sides are now fully reset


async def uio_driver(dut, ls, rng, loop_p=0.5, flip_p=0.05):
    """Random external pin levels with partial loopback; changes on rising edges."""
    ext = 0xFF
    while True:
        await RisingEdge(dut.clk)
        if rng.random() < flip_p:
            ext ^= 1 << rng.randrange(8)
        loop = ls.last_line
        mix = rng.getrandbits(8) if rng.random() < 0.01 else 0
        val = ext
        if rng.random() < loop_p:
            val = (ext & 0xF0) | ((loop << 1) & 0x0E) | (ext & 1)  # P0->P1, P1->P2, P2->P3
        dut.uio_in.value = (val ^ mix) & 0xFF


def rand_instr(rng):
    pin = rng.randrange(8) if rng.random() < 0.9 else rng.randrange(16)
    dly = rng.randrange(8) if rng.random() < 0.5 else 0
    tgt = rng.randrange(64)
    op = rng.choice(["set", "set", "setoe", "out", "out", "in", "in", "wait", "jpin", "jmp", "jmp",
                     "djnz", "mov", "delay", "pull", "push", "vote", "nop", "illegal"])
    if op == "set":
        return I.set_(pin, rng.randrange(4), dly)
    if op == "setoe":
        return I.setoe(pin, rng.getrandbits(1), dly)
    if op == "out":
        return I.out(pin, rng.randint(1, 8), dly)
    if op == "in":
        return I.in_(pin, rng.randint(1, 16), dly)
    if op == "wait":
        return I.wait(pin, rng.getrandbits(1), rng.choice([None, 0, 1]), dly)
    if op == "jpin":
        return I.jpin(pin, rng.getrandbits(1), tgt, dly)
    if op == "jmp":
        return I.jmp(tgt, rng.randrange(8), dly)
    if op == "djnz":
        return I.djnz(rng.getrandbits(1), tgt, dly)
    if op == "mov":
        return I.mov(rng.getrandbits(1), rng.randint(0, 25), dly)
    if op == "delay":
        return I.delay(rng.randint(0, 40), dly)
    if op == "pull":
        return I.pull(rng.getrandbits(1), dly)
    if op == "push":
        return I.push(rng.getrandbits(1), dly)
    if op == "vote":
        return I.vote(pin, dly)
    if op == "nop":
        return I.nop(dly)
    return (rng.randint(14, 31) << 19) | rng.getrandbits(19)


async def random_config(bus, rng, cfg):
    await bus.write(R.PIN_OWNER, rng.getrandbits(8))
    await bus.write(R.PIN_OD, rng.getrandbits(8) if rng.random() < 0.5 else 0)
    for c in range(cfg.num_ctx):
        await bus.setup_ctx(c, start_pc=rng.randrange(64), msb=rng.getrandbits(1),
                            vote=(rng.randint(0, 9), rng.randint(0, 3), rng.randint(0, 6)))


async def phase_a(dut, ls, bus, rng, ops):
    cfg = ls.cfg
    await bus.load_program([rand_instr(rng) for _ in range(cfg.imem_depth)])
    await random_config(bus, rng, cfg)
    await bus.write(R.RESTART, 0b11)
    await bus.write(R.RUN, rng.choice([1, 2, 3, 3]))
    for _ in range(ops):
        a = rng.random()
        if a < 0.25:
            await bus.write(R.TXF, rng.getrandbits(8))
        elif a < 0.40:
            await bus.read(R.RXF)
        elif a < 0.50:
            await bus.read(R.STATUS)
        elif a < 0.58:
            await bus.read(rng.choice([R.FIFO_CNT, R.PIN_IN, R.IMEM_ADDR, R.ctx_reg(0, R.PC),
                                       R.ctx_reg(1, R.CSTAT), rng.getrandbits(8)]))
        elif a < 0.64:
            await bus.write(R.RUN, rng.getrandbits(2))
        elif a < 0.68:
            await bus.write(R.RESTART, rng.getrandbits(2))
        elif a < 0.72:
            await bus.write(R.ERR_CLR, rng.getrandbits(8))
        elif a < 0.75:
            await bus.write(R.IRQ_EN, rng.getrandbits(3))
        elif a < 0.80:  # IMEM commit: rejected if running, applied if halted
            await bus.write(R.IMEM_ADDR, rng.randrange(64))
            w = rand_instr(rng)
            for b in range(3):
                await bus.write(R.IMEM_DATA + b, (w >> (8 * b)) & 0xFF)
        elif a < 0.84:
            await random_config(bus, rng, cfg)
        elif a < 0.86:
            await bus.op(R.OP_SYNC)
        else:
            await ClockCycles(dut.clk, rng.randint(1, 60))


async def phase_b(dut, rng, cycles):
    for _ in range(cycles):
        await RisingEdge(dut.clk)
        dut.ui_in.value = rng.getrandbits(8)


async def random_lockstep(dut, seed):
    rng = random.Random(seed)
    ls = Lockstep(dut)
    await start(dut, ls)
    cocotb.start_soon(uio_driver(dut, ls, rng))
    bus = R.HostBus(dut)
    await phase_a(dut, ls, bus, rng, ops=400)
    await phase_b(dut, rng, 8000)
    dut._log.info(f"seed {seed}: {ls.compared} cycles compared, "
                  f"{len(ls.internal)} internal signals + 3 output ports per cycle")
    assert ls.compared > 10000


@cocotb.test()
async def lockstep_seed1(dut):
    await random_lockstep(dut, 1)


@cocotb.test()
async def lockstep_seed2(dut):
    await random_lockstep(dut, 2)


@cocotb.test()
async def lockstep_seed3(dut):
    await random_lockstep(dut, 3)


@cocotb.test()
async def lockstep_uart(dut):
    """Real firmware in lockstep: UART TX on ctx0, vote RX on ctx1, P0 looped to P1."""
    ls = Lockstep(dut)
    await start(dut, ls)
    rng = random.Random(7)

    async def loop():
        while True:
            await RisingEdge(dut.clk)
            dut.uio_in.value = 0xFD | ((ls.last_line & 1) << 1)

    cocotb.start_soon(loop())
    bus = R.HostBus(dut)
    N, S = 9, 12
    B = pe_fw.rx_bit_period(N, S)
    await bus.load_program(pe_fw.uart_tx(0, B, base=0) + [I.jmp(12)] * 8, start=0)
    await bus.load_program(pe_fw.uart_rx(1, N, S, base=20) + [I.jmp(32)] * 32, start=20)
    await bus.write(R.PIN_OWNER, 0b10)
    await bus.setup_ctx(0, start_pc=0)
    await bus.setup_ctx(1, start_pc=20, vote=(N, 1, S))
    await bus.write(R.RESTART, 0b11)
    await bus.write(R.RUN, 0b11)
    data = [rng.getrandbits(8) for _ in range(6)]
    got, todo = [], list(data)
    while len(got) < len(data):
        st = await bus.read(R.STATUS)
        if not st & R.ST_RXF_EMPTY:
            got.append(await bus.read(R.RXF))
        elif todo and not st & R.ST_TXF_FULL:
            await bus.write(R.TXF, todo.pop(0))
    assert got == data
    dut._log.info(f"uart lockstep: {ls.compared} cycles compared")
