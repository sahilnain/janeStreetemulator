# SPDX-License-Identifier: Apache-2.0
"""
test_lockstep: random programs on the RTL and the Python model, compared every cycle.

What this file does
-------------------
The protocol tests can run in lockstep too (make LOCKSTEP=1), but real firmware only
uses a small part of the state space. Here both sides run random programs with random
unit, detector and CRC configurations, random pins and random host traffic. Every
falling edge compares the pins and the internal state (common/lockstep.py).

Stimulus
--------
* Phase A (protocol): loads a program, fills DMEM, sets random owners and open-drain bits,
  starts the controller, then performs random host operations: FIFO traffic, status reads,
  RUN / RESTART toggles, error clears, IRQ enables, doorbells, DMEM accesses. The program is
  either chaos (every opcode, reserved ones too, with jump targets inside IMEM) or structured
  scenarios that configure the units at random and run packets to their end or abort them.
* Phase B (raw): ui_in is random every cycle, so the host-bus protocol is broken on purpose.
* The uio pins get random external levels and partial loopback.

Config writes to a running unit are unspecified (blocks.md Section 2.4), so test_fuzz_*
only reconfigures idle units: each `ucfg` is preceded by a `ustop` of both units. The test
test_fuzz_reconfig_busy does not do that; it checks that the RTL matches the model there too.
"""

import os
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "common"))

import cocotb  # noqa: E402
import pe_host as R  # noqa: E402
import pe_isa as I  # noqa: E402
from cocotb.clock import Clock  # noqa: E402
from cocotb.triggers import ClockCycles, RisingEdge  # noqa: E402
from lockstep import Lockstep  # noqa: E402

DEPTH = R.IMEM_WORDS
FAST_B = (1, 2, 3, 4, 5, 8, 12, 16)


def rand_cfg_value(rng, u, idx):
    """A config value that makes short packets, so random programs see many of them."""
    pin = lambda: rng.randrange(8) if rng.random() < 0.95 else rng.randrange(16)  # noqa: E731
    if u == I.TX:
        if idx == I.TX_PINS:
            return I.tx_pins(pin(), rng.getrandbits(1), pin(), rng.getrandbits(1))
        if idx == I.TX_BIT:
            return rng.choice(FAST_B)
        if idx == I.TX_FRAME:
            return I.tx_frame(rng.getrandbits(1), rng.getrandbits(1), rng.randrange(20), rng.getrandbits(1))
        if idx == I.TX_CLK:
            return rng.getrandbits(3)
        if idx == I.TX_TICK:
            return I.tx_tick(rng.getrandbits(1) if rng.random() < 0.4 else 0, pin(), rng.getrandbits(1), rng.getrandbits(1))
        return I.tx_line(rng.getrandbits(1), rng.getrandbits(1), rng.getrandbits(1), pin(),
                         rng.choice((0, 0, 1, 2, 3, 6)))
    if u == I.RX:
        if idx == I.RX_PINS:
            return I.rx_pins(pin(), pin(), rng.getrandbits(1), pin(), rng.getrandbits(1))
        if idx == I.RX_BIT:
            return rng.choice(FAST_B)
        if idx == I.RX_FRAME:
            return I.rx_frame(rng.getrandbits(1), rng.getrandbits(1), rng.getrandbits(1), rng.getrandbits(1),
                              rng.randrange(20))
        if idx == I.RX_VOTE:
            return I.rx_vote(rng.randrange(8), rng.randrange(4))
        if idx == I.RX_STEP:
            return rng.randrange(4)
        if idx == I.RX_FIRST:
            return rng.randrange(12)
        if idx == I.RX_TICK:
            return I.rx_tick(rng.randrange(4), pin(), rng.getrandbits(1), rng.randrange(6))
        return I.rx_line(rng.randrange(4), rng.getrandbits(1), rng.getrandbits(1), rng.choice((0, 0, 1, 2, 6)))
    if u == I.CRC:
        if idx == I.CRC_CTRL:
            return I.crc_ctrl(rng.randrange(3), rng.getrandbits(1), rng.choice((0, 0, 1, 3, 40)))
        return rng.getrandbits(16)
    if idx == I.D_COND:
        return I.cond(pin(), rng.randrange(4), rng.getrandbits(1), rng.getrandbits(1), rng.getrandbits(1), pin(),
                      rng.getrandbits(1), rng.getrandbits(1), rng.getrandbits(1))
    if idx == I.D_DUR:
        return rng.randrange(8)
    if idx == I.D_AFTER:
        return rng.randrange(4)
    return rng.randrange(8)


def rand_program(rng, safe):
    a = I.Asm()
    words = 0
    tgt = lambda: rng.randrange(DEPTH)                                             # noqa: E731
    pin = lambda: rng.randrange(8) if rng.random() < 0.95 else rng.randrange(16)   # noqa: E731
    dly = lambda: rng.randrange(8) if rng.random() < 0.25 else 0                   # noqa: E731
    reg = lambda: rng.randrange(8)                                                 # noqa: E731
    op2 = lambda: I.R(reg()) if rng.random() < 0.4 else rng.getrandbits(16)        # noqa: E731
    small = lambda: I.R(reg()) if False else rng.randrange(64)                     # noqa: E731
    while words < DEPTH - 1:
        k = rng.choice(("ucfg", "ucfg", "ucfg", "ustart", "ustart", "uwait", "ustop", "urd", "evt", "evt", "set",
                        "setoe", "wait", "jpin", "jmp", "jmp", "djnz", "call", "ret", "delay", "alu", "alu", "shift",
                        "par", "rdpins", "bitin", "bitout", "fget", "fput", "ld", "st", "host", "nop", "illegal"))
        n0 = len(a.words())
        if k == "ucfg":
            u = rng.randrange(5)
            n_idx = {I.TX: 6, I.RX: 8, I.CRC: 3}.get(u, 4)
            idx = rng.randrange(n_idx)
            if safe:
                a.ustop(I.M_BOTH)
            a.ucfg(u, idx, rand_cfg_value(rng, u, idx), dly=dly())
        elif k == "ustart":
            a.ustart(rng.randrange(4), None if rng.random() < 0.4 else rng.getrandbits(16), dly=dly())
        elif k == "uwait":
            a.uwait(rng.randrange(4), dly=dly())
        elif k == "ustop":
            a.ustop(rng.randrange(4), dly=dly())
        elif k == "urd":
            a.urd(reg(), rng.randrange(6), rng.randrange(3), dly=dly())
        elif k == "evt":
            a.evt(rng.getrandbits(1), rng.randrange(5), dly=dly())
        elif k == "set":
            a.set(pin(), rng.randrange(4), dly=dly())
        elif k == "setoe":
            a.setoe(pin(), rng.getrandbits(1), dly=dly())
        elif k == "wait":
            if rng.random() < 0.8:
                a.mov(1, rng.randrange(40))
                a.wait(pin(), rng.getrandbits(1), rd=1, dly=dly())
            else:
                a.wait(pin(), rng.getrandbits(1), dly=dly())
        elif k == "jpin":
            a.jpin(pin(), rng.getrandbits(1), tgt(), dly=dly())
        elif k == "jmp":
            a.jmp(tgt(), rng.randrange(32), dly=dly())
        elif k == "djnz":
            a.djnz(reg(), tgt(), dly=dly())
        elif k == "call":
            a.call(tgt(), dly=dly())
        elif k == "ret":
            a.ret(dly=dly())
        elif k == "delay":
            a.delay(small(), dly=dly())
        elif k == "alu":
            a.alu(rng.choice(("mov", "add", "sub", "and_", "or_", "xor", "cmp")), reg(), op2(), dly=dly())
        elif k == "shift":
            (a.shl if rng.getrandbits(1) else a.shr)(reg(), rng.randrange(16), dly=dly())
        elif k == "par":
            a.par(reg(), dly=dly())
        elif k == "rdpins":
            a.rdpins(reg(), dly=dly())
        elif k == "bitin":
            a.bitin(reg(), pin(), rng.getrandbits(1), dly=dly())
        elif k == "bitout":
            a.bitout(pin(), reg(), rng.getrandbits(1), dly=dly())
        elif k == "fget":
            a.fget(reg(), rng.getrandbits(1), rng.getrandbits(1), dly=dly())
        elif k == "fput":
            a.fput(reg(), rng.getrandbits(1), rng.getrandbits(1), dly=dly())
        elif k == "ld":
            a.ld(reg(), reg(), rng.getrandbits(1), dly=dly())
        elif k == "st":
            a.st(reg(), reg(), rng.getrandbits(1), dly=dly())
        elif k == "host":
            a.host_irq(dly=dly()) if rng.getrandbits(1) else a.dbclr(dly=dly())
        elif k == "nop":
            a.nop(dly=dly())
        else:
            a.raw((rng.choice(I.RESERVED_OPS) << 26) | rng.getrandbits(26))
        words = len(a.words())
        assert words > n0
    w = a.words()[:DEPTH - 1]
    # A delay that came out of a register could stall for a long time; RUN and RESTART end it.
    return w + [I.word(I.OP["jmp"], imm=rng.randrange(DEPTH))]              # the last word never falls off IMEM


def rand_scenarios(rng):
    """Structured: configure the units at random, run packets to their end or abort them at a
    random moment, look at the results, and go on to the next scenario. Unlike rand_program, most
    packets finish, so the end-of-packet paths and the after-steps get used."""
    a = I.Asm()
    while len(a.words()) < DEPTH - 30:
        a.ustop(I.M_BOTH).evt(I.DET_TX, I.EV_DISARM).evt(I.DET_RX, I.EV_DISARM)
        for u, regs in ((I.TX, 6), (I.RX, 8), (I.ETX_U, 4), (I.ERX_U, 4), (I.CRC, 3)):
            for idx in range(regs):
                if rng.random() < 0.75:
                    v = rand_cfg_value(rng, u, idx)
                    if u in (I.TX, I.RX) and idx == 1:
                        v = rng.choice((2, 3, 4, 5, 7, 8))
                    if u == I.TX and idx == I.TX_TICK and rng.random() < 0.7:
                        v &= ~1                                 # mostly the own timer, so packets end
                    a.ucfg(u, idx, v)
        p = rng.randrange(4)
        a.setoe(p, 1).evt(I.DET_TX, I.EV_CLR).evt(I.DET_RX, I.EV_CLR)
        if rng.getrandbits(1):
            a.evt(I.DET_RX, I.EV_ARM)
        if rng.getrandbits(1):
            a.evt(I.DET_TX, I.EV_ARM)
        for _ in range(rng.randint(1, 3)):
            a.ustart(rng.choice((1, 2, 3, 3)), rng.getrandbits(16) if rng.random() < 0.7 else None)
            a.delay(rng.choice((0, 3, 8, 20, 40, 90, 200)))
            k = rng.random()
            if k < 0.3:
                a.ustop(rng.randrange(1, 4))
            a.urd(0, I.RX, I.RX_STAT).urd(1, I.RX, I.RX_DATA).urd(2, I.TX, 0).urd(3, I.CRC, 0)
            a.urd(4, I.ETX_U, 0).urd(5, I.ERX_U, 0)
            a.fget(6, I.FIFO_RX, nb=1)
    w = a.words()[:DEPTH - 1]                     # a scenario cut short is still a valid program
    return w + [I.word(I.OP["jmp"], imm=0)] * (DEPTH - len(w))


async def host_ops(dut, ls, bus, rng, n):
    for _ in range(n):
        a = rng.random()
        if a < 0.22:
            await bus.write(R.TXF, rng.getrandbits(8))
        elif a < 0.36:
            await bus.read(R.RXF)
        elif a < 0.46:
            await bus.read(R.STATUS)
        elif a < 0.56:
            await bus.read(rng.choice([R.STATUS2, R.FIFO_CNT, R.PIN_IN, R.IMEM_ADDR, R.PC, R.CSTAT, R.DMEM_ADDR,
                                       R.DMEM_DATA, rng.getrandbits(8)]))
        elif a < 0.60:
            await bus.write(R.RUN, rng.getrandbits(1))
        elif a < 0.635:
            await bus.write16(R.START_PC, rng.randrange(DEPTH))
            await bus.write(R.RESTART, 1)
        elif a < 0.67:
            await bus.write(R.ERR_CLR, rng.getrandbits(8))
        elif a < 0.70:
            await bus.write(R.STATUS2, rng.getrandbits(8))
        elif a < 0.73:
            await bus.write(R.IRQ_EN, rng.getrandbits(4))
        elif a < 0.76:
            await bus.write(R.DOORBELL, 1)
        elif a < 0.80:
            await bus.write(R.DMEM_ADDR, rng.getrandbits(8))
            await bus.write(R.DMEM_DATA, rng.getrandbits(8))
        elif a < 0.84:
            await bus.write(R.PIN_OWNER + rng.randrange(2), rng.getrandbits(8))
        elif a < 0.86:
            await bus.write(R.PIN_OD, rng.getrandbits(8) if rng.random() < 0.3 else 0)
            await bus.write(R.PIN_FILT, rng.getrandbits(8) if rng.random() < 0.3 else 0)
        elif a < 0.87:
            await bus.write(R.RUN, 1)
            await bus.write(R.IMEM_DATA + 3, rng.getrandbits(8))   # commit while running: imem_err
        else:
            await ClockCycles(dut.clk, rng.randint(1, 120))


async def start(dut, ls):
    cocotb.start_soon(Clock(dut.clk, 20834, unit="ps").start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0xFF
    dut.rst_n.value = 0
    await RisingEdge(dut.clk)
    ls.task = cocotb.start_soon(ls.run())
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


async def uio_driver(dut, up, rng):
    """Random external levels with partial loopback; changes on rising edges."""
    ext = 0xFF
    while True:
        await RisingEdge(dut.clk)
        if rng.random() < 0.08:
            ext ^= 1 << rng.randrange(8)
        out, oe = dut.uio_out.value, dut.uio_oe.value
        val = ext
        if out.is_resolvable and oe.is_resolvable and rng.random() < 0.7:
            line = (int(out) & int(oe)) | (ext & ~int(oe))
            val = (ext & 0xE0) | ((line << 1) & 0x1E) | (ext & 1)          # P0->P1 ... P3->P4
        dut.uio_in.value = val & 0xFF


async def fuzz(dut, seed, safe, programs=3, ops=120, raw=2000, fresh=True):
    rng = random.Random(seed)
    ls = Lockstep(dut)
    if fresh:
        await start(dut, ls)
    else:
        dut.ui_in.value = 0
        dut.rst_n.value = 0
        await RisingEdge(dut.clk)
        ls.task = cocotb.start_soon(ls.run())
        await ClockCycles(dut.clk, 10)
        dut.rst_n.value = 1
        await ClockCycles(dut.clk, 5)
    bus = R.HostBus(dut)
    await bus.write(R.DMEM_ADDR, 0)
    await bus.set_addr(R.DMEM_DATA)
    for i in range(256):
        await bus.write_same(rng.getrandbits(8))
    for _ in range(programs):
        await bus.write(R.RUN, 0)
        await bus.load_program(rand_scenarios(rng) if rng.random() < 0.6 else rand_program(rng, safe))
        if not ls.armed:
            ls.arm()
            ls.driver = cocotb.start_soon(uio_driver(dut, None, rng))
        await bus.owners(rng.getrandbits(16))
        await bus.start(rng.randrange(DEPTH))
        await host_ops(dut, ls, bus, rng, ops)
    await bus.write(R.RUN, 0)
    assert ls.compared > 5000
    dut._log.info(f"seed {seed}: {ls.compared} cycles compared on {len(ls.internal)} internal signals and the pins")
    for _ in range(raw):
        await RisingEdge(dut.clk)
        dut.ui_in.value = rng.getrandbits(8) & (0xBF if safe else 0xFF)         # bit 6 is the strobe
    assert ls.compared > 5000 + raw
    ls.stop()
    dut.ui_in.value = 0
    await ClockCycles(dut.clk, 8)


@cocotb.test(skip="FUZZ_SEEDS" not in os.environ)
async def test_fuzz_soak(dut):
    """make FUZZ_SEEDS=first,count [FUZZ_BUSY=1]: many seeds, stops at the first difference."""
    first, count = map(int, os.environ["FUZZ_SEEDS"].split(","))
    for seed in range(first, first + count):
        dut._log.info(f"soak seed {seed}")
        await fuzz(dut, seed, "FUZZ_BUSY" not in os.environ, programs=4, ops=200, raw=1000, fresh=seed == first)


@cocotb.test()
async def test_fuzz_1(dut):
    await fuzz(dut, 1, True)


@cocotb.test()
async def test_fuzz_2(dut):
    await fuzz(dut, 2, True)


@cocotb.test()
async def test_fuzz_3(dut):
    await fuzz(dut, 3, True)


@cocotb.test()
async def test_fuzz_reconfig_busy(dut):
    await fuzz(dut, 4, False)
