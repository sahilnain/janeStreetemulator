# SPDX-License-Identifier: Apache-2.0
"""
test_fabric.py: chip-level corners the protocol tests do not reach (run through test/tb.v).

What this file does
-------------------
The protocol tests use a few fixed pins and short programs. These tests were
added from the coverage report: each one exercises chip wiring or a value range
that no other test toggled.

Tests
-----
1. test_every_pin_units      TX DPIN, TX D2PIN, RX IPIN, RX OPIN and ETX release on all
                             8 pins, with the exact bit times on the wire.
2. test_etx_drive_every_pin  ETX drive 0 / drive 1 after a packet on all 8 pins.
3. test_every_pin_controller set / setoe, open drain and PIN_OD on all 8 pins.
4. test_pin_index_range      a pin index of 8 or more writes nothing and reads 0.
5. test_rx_fifo_overrun      the RX unit overruns RXF: 8 bytes kept in order, the sticky
                             flag and the IRQ set, and cleared through ERR_CLR.
6. test_etx_stop             ETX with STOP ends a TX packet at the chip level.
7. test_long_delay           `delay` with bits 8-15 set, immediate and register: exact time.
8. test_start_pc_deep_stack  START_PC with high bits, 4 nested calls at high addresses.
9. test_erx_arm_pending      ERX status read through `urd`: arm pending while RX is busy.
10. test_pc_past_imem        pc and IMEM_ADDR past IMEM_DEPTH: nops, wrap to 0, writes ignored.
11. test_protocol_switches   what the host and firmware can read shows the features of each
                             protocol present or absent, as built (blocks.md Section 13).
12. test_one_instruction_per_cycle  the pipelined fetch: one instruction a cycle in straight
                             code, through a taken branch, call and ret, and a blocked wait
                             retries every cycle (seen on the pins, cycle by cycle).
13. test_memories            IMEM and DMEM through the chip, with every word and byte used:
                             all 162 IMEM words hold a program whose result depends on every bit
                             of every word (loaded twice, so every word is also rewritten); all
                             256 DMEM bytes written by the host, inverted by the firmware with a
                             read straight after each write, and read back by the host, while the
                             host also reads DMEM during the run (arbitration). Run with and
                             without MEM_CLOCK_GATE.
"""

import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "common"))

import cocotb  # noqa: E402
import pe_build  # noqa: E402
import pe_host as R  # noqa: E402
import pe_isa as I  # noqa: E402
from chip import Chip  # noqa: E402
from cocotb.triggers import ClockCycles, FallingEdge  # noqa: E402
from pe_isa import Asm  # noqa: E402

B = 8
DATA = 0xA5  # LSB first: 1 0 1 0 0 1 0 1


def runs(trace):
    """Run lengths of a trace, as (level, length), without the first and last run."""
    out = []
    for lv in trace:
        if out and out[-1][0] == lv:
            out[-1][1] += 1
        else:
            out.append([lv, 1])
    return [tuple(r) for r in out[1:-1]]


# Bit 0 is high like the idle level and the last bit is high like the level after the
# packet, so the inner runs of DATA are:
WIRE = [(0, B), (1, B), (0, 2 * B), (1, B), (0, B)]
# The inverse on D2PIN goes low at the start and is released (pull-up) at the end:
WIRE_D2 = [(0, B), (1, B), (0, B), (1, 2 * B), (0, B), (1, B), (0, B)]


def rx_cfg(a, ipin, opin=0, oen=0, hold=0, src=I.SRC_OWN):
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(ipin, opin, oen))
    a.ucfg(I.RX, I.RX_BIT, B)
    a.ucfg(I.RX, I.RX_VOTE, I.rx_vote(1))
    a.ucfg(I.RX, I.RX_STEP, 1)
    a.ucfg(I.RX, I.RX_FIRST, B // 2 + 2)            # mid-bit, plus the 2-FF input delay
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(src))
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(hold=hold))


def idle(a):
    return a.L("idle").jmp("idle")


@pe_build.test("usb", "i2c")            # DIFF and release are USB; RX drive mode is I2C
async def test_every_pin_units(dut):
    for p in range(8):
        q, d2, r = (p + 1) % 8, (p + 3) % 8, (p + 5) % 8
        chip = Chip(dut)
        await chip.reset()
        chip.loop.append((p, q))
        a = Asm()
        a.ucfg(I.TX, I.TX_PINS, I.tx_pins(p))
        a.ucfg(I.TX, I.TX_BIT, B)
        a.ucfg(I.TX, I.TX_LINE, I.tx_line(diff=1, d2pin=d2))
        a.ucfg(I.ETX_U, I.D_AFTER, I.A_RELEASE)
        rx_cfg(a, q, opin=r, oen=1)
        a.setoe(p, 1).setoe(d2, 1).setoe(r, 1)
        a.ustart(I.M_BOTH, DATA).uwait(I.M_BOTH)
        idle(a)
        await chip.bus.load_program(a.words())
        await chip.bus.owners(R.owner_map(**{f"p{p}": R.OWN_TX, f"p{d2}": R.OWN_TX, f"p{r}": R.OWN_RX}))
        await chip.bus.start()
        await ClockCycles(dut.clk, 40 + 12 * B)
        assert await chip.bus.read(R.RXF) == DATA, f"pin {p}: RX on pin {q}"
        assert runs(chip.trace(p)) == WIRE, f"pin {p}: {runs(chip.trace(p))}"
        assert runs(chip.trace(d2)) == WIRE_D2, f"D2 pin {d2}: {runs(chip.trace(d2))}"
        assert runs(chip.trace(r)) == WIRE, f"RX OPIN {r}: {runs(chip.trace(r))}"
        oe = int(dut.uio_oe.value)
        assert not oe & ((1 << p) | (1 << d2)), f"ETX release, pins {p} and {d2}: oe {oe:02x}"
        assert oe & (1 << r), f"RX OPIN {r} stays driven: oe {oe:02x}"
        assert not await chip.bus.read(R.STATUS) & R.ST_OWNER_ERR
        await chip.bus.write(R.RUN, 0)


@cocotb.test()
async def test_etx_drive_every_pin(dut):
    for p in range(8):
        for action, level, data in ((I.A_DRIVE0, 0, 0xFF), (I.A_DRIVE1, 1, 0x00)):
            chip = Chip(dut)
            await chip.reset()
            a = Asm()
            a.ucfg(I.TX, I.TX_PINS, I.tx_pins(p))
            a.ucfg(I.TX, I.TX_BIT, 4)
            a.ucfg(I.ETX_U, I.D_AFTER, action)
            a.ustart(I.M_TX, data).uwait(I.M_TX)
            idle(a)
            await chip.bus.load_program(a.words())
            await chip.bus.owners(R.owner_map(**{f"p{p}": R.OWN_TX}))
            await chip.bus.start()
            await ClockCycles(dut.clk, 80)
            assert (int(dut.uio_out.value) >> p) & 1 == level, f"pin {p}, level {level}"
            assert (int(dut.uio_oe.value) >> p) & 1 == 1, f"pin {p}: ETX drive sets oe"
            await chip.bus.write(R.RUN, 0)


@cocotb.test()
async def test_every_pin_controller(dut):
    for p in range(8):
        mask = 1 << p
        chip = Chip(dut)
        await chip.reset()
        a = Asm().setoe(p, 1).delay(6).L("l").set(p, I.SRC_0).delay(6).set(p, I.SRC_1).delay(6).jmp("l")
        await chip.bus.load_program(a.words())
        await chip.bus.start()
        seen = set()
        for _ in range(60):
            await FallingEdge(dut.clk)
            seen.add(int(dut.uio_out.value) & mask)
        assert seen == {0, mask}, f"pin {p}: out {seen}"
        assert int(dut.uio_oe.value) == mask, f"pin {p}: oe {int(dut.uio_oe.value):02x}"
        await chip.bus.write(R.RUN, 0)

        await chip.bus.write(R.PIN_OD, mask)
        if not pe_build.has("i2c"):                 # open drain is part of I2C (blocks.md Section 13)
            assert await chip.bus.read(R.PIN_OD) == 0
            continue
        assert await chip.bus.read(R.PIN_OD) == mask
        await chip.bus.start()
        oes = set()
        for _ in range(60):
            await FallingEdge(dut.clk)
            assert int(dut.uio_out.value) & mask == 0, f"pin {p}: open drain drives high"
            oes.add(int(dut.uio_oe.value) & mask)
        assert oes == {0, mask}, f"pin {p}: open-drain oe {oes}"
        await chip.bus.write(R.RUN, 0)
        await chip.bus.write(R.PIN_OD, 0)


@cocotb.test()
async def test_pin_index_range(dut):
    # the vote path, and the tick path if SPI or I2C is built (blocks.md Section 13)
    for src in (I.SRC_OWN, I.SRC_TXTICK) if (pe_build.has("spi") or pe_build.has("i2c")) else (I.SRC_OWN,):
        chip = Chip(dut)
        await chip.reset()
        a = Asm()
        a.ucfg(I.TX, I.TX_PINS, I.tx_pins(8))
        a.ucfg(I.TX, I.TX_BIT, B)
        rx_cfg(a, 8, src=src)                       # all pads read 1 here: 0 means no alias
        a.ustart(I.M_BOTH, 0x00).uwait(I.M_BOTH)
        idle(a)
        await chip.bus.load_program(a.words())
        await chip.bus.owners(R.owner_map(**{f"p{i}": R.OWN_TX for i in range(8)}))
        await chip.bus.start()
        await ClockCycles(dut.clk, 40 + 12 * B)
        assert not await chip.bus.read(R.STATUS) & R.ST_RXF_EMPTY, f"src {src}: no byte received"
        assert await chip.bus.read(R.RXF) == 0x00, f"src {src}: pin 8 must read 0"
        assert all(s == 0xFF for s in chip.samples), "pin 8 must write nothing"
        assert int(dut.uio_oe.value) == 0
        assert not await chip.bus.read(R.STATUS) & R.ST_OWNER_ERR
        await chip.bus.write(R.RUN, 0)


@cocotb.test()
async def test_rx_fifo_overrun(dut):
    chip = Chip(dut)
    await chip.reset()
    chip.loop.append((0, 1))
    data = [0x11 * (i + 1) for i in range(9)]
    a = Asm()
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(0))
    a.ucfg(I.TX, I.TX_BIT, B)
    rx_cfg(a, 1)
    a.setoe(0, 1)
    for d in data:
        a.ustart(I.M_BOTH, d).uwait(I.M_BOTH)
    idle(a)
    await chip.bus.load_program(a.words())
    await chip.bus.owners(R.owner_map(p0=R.OWN_TX))
    await chip.bus.write(R.IRQ_EN, 0x04)
    assert chip.bus.irq() == 0
    await chip.bus.start()
    await ClockCycles(dut.clk, 9 * (10 * B + 20))
    st = await chip.bus.read(R.STATUS)
    assert st & R.ST_RXF_FULL and st & R.ST_RXF_OVR, f"STATUS {st:02x}"
    assert chip.bus.irq() == 1
    assert await chip.bus.read(R.FIFO_CNT) == 0x80
    got = [await chip.bus.read(R.RXF) for _ in range(8)]
    assert got == data[:8], f"the ninth byte is the one dropped: {got}"
    assert await chip.bus.read(R.STATUS) & R.ST_RXF_OVR, "the flag is sticky"
    await chip.bus.write(R.ERR_CLR, 0x20)
    assert not await chip.bus.read(R.STATUS) & R.ST_RXF_OVR
    assert chip.bus.irq() == 0
    await chip.bus.write(R.IRQ_EN, 0)
    await chip.bus.write(R.RUN, 0)


@pe_build.test("usb")                   # STOP on a detector is a USB feature
async def test_etx_stop(dut):
    chip = Chip(dut)
    await chip.reset()
    bit = 20
    a = Asm()
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(0))
    a.ucfg(I.TX, I.TX_BIT, bit)
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(nbits=16))
    a.ucfg(I.ETX_U, I.D_COND, I.cond(7, I.LEVEL, 0, stop=1))
    a.ucfg(I.ETX_U, I.D_DUR, 1)
    a.setoe(0, 1).setoe(6, 1).set(6, I.SRC_1)
    a.evt(I.DET_TX, I.EV_ARM)
    a.ustart(I.M_TX, 0x5555).uwait(I.M_TX)
    a.jmp("fired", I.ETX)
    idle(a)
    a.L("fired").set(6, I.SRC_0)                    # marker: packet over and the flag is set
    a.L("end").jmp("end")
    await chip.bus.load_program(a.words())
    await chip.bus.owners(R.owner_map(p0=R.OWN_TX))
    await chip.bus.start()
    await ClockCycles(dut.clk, 40)
    while sum(runs_all(chip.trace(0))) < 4:         # the packet runs, then the pin goes low
        await FallingEdge(dut.clk)
    t = len(chip.samples)
    chip.drive[7] = 0
    await ClockCycles(dut.clk, 3 + 4 * bit)
    # 2-FF input delay, then the stop: nothing is written after it
    after = chip.trace(0, t + 4)
    assert len(set(after)) == 1, f"TX pin moved after the stop: {after[:40]}"
    assert chip.trace(6)[-1] == 0, "uwait must end, with the ETX flag set"
    assert 16 * bit > t - first_edge(chip.trace(0)), "the stop must come before the natural end"
    await chip.bus.write(R.RUN, 0)


def runs_all(trace):
    return [1 for x, y in zip(trace, trace[1:]) if x != y]


def first_edge(trace):
    return next(i for i, (x, y) in enumerate(zip(trace, trace[1:])) if x != y)


@cocotb.test()
async def test_long_delay(dut):
    async def low_time(value, reg):
        chip = Chip(dut)
        await chip.reset()
        a = Asm().setoe(0, 1).mov(1, value)
        a.set(0, I.SRC_0)
        a.delay(I.R(1) if reg else value)
        a.set(0, I.SRC_1)
        idle(a)
        await chip.bus.load_program(a.words())
        await chip.bus.start()
        await ClockCycles(dut.clk, 40 + value)
        await chip.bus.write(R.RUN, 0)
        assert chip.trace(0)[-1] == 1, f"delay {value:#x} has not ended"
        return chip.trace(0).count(0)

    for reg in (0, 1):
        base = await low_time(0, reg)
        for value in (0x5500, 0xAA00, 0x0100):
            assert await low_time(value, reg) - base == value, (reg, hex(value))


@cocotb.test()
async def test_start_pc_deep_stack(dut):
    # return addresses that set and clear bits 3, 4, 6 and 7 in every stack slot
    for base in (0x55, 0x81, 0x07, 0x00):
        chip = Chip(dut)
        await chip.reset()
        a = Asm(base)
        a.mov(0, 1).call("f1").fput(0, I.FIFO_RX)
        a.L("end").jmp("end")
        for name, nxt, k in (("f1", "f2", 1), ("f2", "f3", 2), ("f3", "f4", 3)):
            a.L(name).call(nxt).shl(0, 2).or_(0, k).ret()
        a.L("f4").ret()
        await chip.bus.write(R.RUN, 0)
        await chip.bus.load_program(a.words(), start=base)
        await chip.bus.start(base)
        await ClockCycles(dut.clk, 120)
        assert await chip.bus.read(R.RXF) == 121, "returns in reverse order: 1, 7, 30, 121"
        assert not await chip.bus.read(R.STATUS2) & R.S2_STK
        await chip.bus.write(R.RUN, 0)

    chip = Chip(dut)
    await chip.reset()
    top = R.IMEM_WORDS - 1                          # the last word holds the only instruction
    b = Asm(top).L("self").jmp("self")
    await chip.bus.load_program(b.words(), start=top)
    await chip.bus.start(top)
    await ClockCycles(dut.clk, 30)
    assert await chip.bus.read16(R.START_PC) == top
    assert not await chip.bus.read(R.STATUS2) & R.S2_ILL
    await chip.bus.write(R.RUN, 0)


@cocotb.test()
async def test_erx_arm_pending(dut):
    chip = Chip(dut)
    await chip.reset()
    a = Asm()
    rx_cfg(a, 1)
    a.ustart(I.M_RX).evt(I.DET_RX, I.EV_ARM)        # RX is busy: the arm waits
    a.urd(0, I.ERX_U, 0).fput(0, I.FIFO_RX)
    a.uwait(I.M_RX).delay(3)
    a.urd(0, I.ERX_U, 0).fput(0, I.FIFO_RX)         # RX is idle: armed
    idle(a)
    await chip.bus.load_program(a.words())
    await chip.bus.start()
    await ClockCycles(dut.clk, 40 + 12 * B)
    got = [await chip.bus.read(R.RXF) for _ in range(3)]
    assert got == [0x04, 0xFF, 0x01], f"status, received byte (pull-ups), status: {got}"
    await chip.bus.write(R.RUN, 0)


@cocotb.test()
async def test_pc_past_imem(dut):
    chip = Chip(dut)
    await chip.reset()
    a = Asm().mov(0, 0x5A).fput(0, I.FIFO_RX).L("e").jmp("e")
    await chip.bus.load_program(a.words())
    # A write past the end must not disturb word 0 (a wrong decode would alias it)
    await chip.bus.write(R.IMEM_ADDR, R.IMEM_WORDS)
    await chip.bus.write(R.IMEM_ADDR_H, 0)
    for b in range(4):
        await chip.bus.write(R.IMEM_DATA + b, 0xFF)
    await chip.bus.start(R.IMEM_WORDS)                 # nops up to 255, then wrap to 0
    await ClockCycles(dut.clk, 2 * (256 - R.IMEM_WORDS) + 40)
    assert await chip.bus.read(R.RXF) == 0x5A, "the program at 0 runs after the wrap"
    st2 = await chip.bus.read(R.STATUS2)
    assert not st2 & R.S2_ILL, f"a nop is not an illegal opcode: {st2:02x}"
    await chip.bus.write(R.RUN, 0)


@cocotb.test()
async def test_protocol_switches(dut):
    """Each build has what it was built with (runs in every build). What can be read shows it: without I2C,
    PIN_OD and PIN_FILT stay 0; without USB, the CRC register stays 0 whatever the firmware writes.
    The TX, RX and detector config registers cannot be read: the lockstep fuzz covers them, since
    the model has the same switches, and so does the z3 proof (model/tests/test_equiv.py)."""
    chip = Chip(dut)
    await chip.reset()
    bus = chip.bus
    for reg in (R.PIN_OD, R.PIN_FILT):
        await bus.write(reg, 0xFF)
        assert await bus.read(reg) == (0xFF if pe_build.has("i2c") else 0), hex(reg)
        await bus.write(reg, 0)
    # the CRC unit: set it up as USB does and read the register back
    a = Asm()
    a.ucfg(I.CRC, I.CRC_CTRL, I.crc_ctrl(I.CRC_TX, append=1)).ucfg(I.CRC, I.CRC_POLY, 0x8005)
    a.ucfg(I.CRC, I.CRC_INIT, 0xFFFF)
    a.urd(0, I.CRC, I.CRC_VAL).fput(0, I.FIFO_RX).shr(0, 8).fput(0, I.FIFO_RX)
    a.L("end").jmp("end")
    await bus.load_program(a.words())
    await bus.start()
    await ClockCycles(dut.clk, 60)
    got = [await bus.read(R.RXF), await bus.read(R.RXF)]
    assert got == ([0xFF, 0xFF] if pe_build.has("usb") else [0, 0]), got
    await bus.write(R.RUN, 0)


def edges(trace, lvl):
    """Cycles where the trace goes to `lvl`."""
    return [i + 1 for i, (x, y) in enumerate(zip(trace, trace[1:])) if x != y and y == lvl]


@cocotb.test()
async def test_one_instruction_per_cycle(dut):
    async def run(a, cycles=120):
        chip = Chip(dut)
        await chip.reset()
        await chip.bus.load_program(a.words())
        await chip.bus.start()
        await ClockCycles(dut.clk, cycles)
        await chip.bus.write(R.RUN, 0)
        return chip

    def spacing(chip, n, period, what):
        falls = edges(chip.trace(0), 0)
        assert len(falls) == n, f"{what}: {len(falls)} falling edges, expected {n}"
        gaps = {y - x for x, y in zip(falls, falls[1:])}
        assert gaps == {period}, f"{what}: falling edges {gaps} cycles apart, expected {period}"

    # 1. Straight code: set 0, set 1, ... toggles the pin every cycle.
    a = Asm().set(0, I.SRC_1).setoe(0, 1)
    for _ in range(20):
        a.set(0, I.SRC_0).set(0, I.SRC_1)
    idle(a)
    spacing(await run(a), 20, 2, "straight code")

    # 2. A taken branch costs one cycle, like any instruction: set, set, djnz = 3 cycles a turn.
    a = Asm().set(0, I.SRC_1).setoe(0, 1).mov(1, 10)
    a.L("l").set(0, I.SRC_0).set(0, I.SRC_1).djnz(1, "l")
    idle(a)
    spacing(await run(a), 10, 3, "djnz loop")

    # 3. call and ret: set 0, call | set 1, ret | djnz = 5 cycles a turn, low for 2 of them.
    a = Asm().set(0, I.SRC_1).setoe(0, 1).mov(1, 10)
    a.L("l").set(0, I.SRC_0).call("f").djnz(1, "l")
    idle(a)
    a.L("f").set(0, I.SRC_1).ret()
    chip = await run(a)
    spacing(chip, 10, 5, "call / ret loop")
    rises = edges(chip.trace(0), 1)
    falls = edges(chip.trace(0), 0)
    lows = [min(r for r in rises if r > f) - f for f in falls]
    assert lows == [2] * 10, f"set 0 and call: low for {lows} cycles, expected 2"

    # 4. A blocked instruction retries every cycle: the time from an input edge to the answer is
    # the same on whichever cycle the edge comes (with an issue every 2 cycles it would alternate).
    lat = set()
    for k in range(4):
        chip = Chip(dut)
        await chip.reset()
        chip.drive[1] = 0
        a = Asm().set(0, I.SRC_0).setoe(0, 1).wait(1, 1).set(0, I.SRC_1)
        idle(a)
        await chip.bus.load_program(a.words())
        await chip.bus.start()
        await ClockCycles(dut.clk, 30 + k)
        t0 = len(chip.samples)
        chip.drive[1] = 1
        await ClockCycles(dut.clk, 20)
        rise = edges(chip.trace(0), 1)
        assert rise, f"edge {k}: the wait never ended"
        lat.add(rise[-1] - t0)
        await chip.bus.write(R.RUN, 0)
    assert len(lat) == 1, f"wait answers after {sorted(lat)} cycles: not one try a cycle"


@cocotb.test()
async def test_memories(dut):
    rng = random.Random(1234)

    # IMEM: every word in use. Words 0..157 xor a random 16-bit value into r1 (0 after RESTART), so a
    # wrong bit anywhere changes the result; 158..160 send r1 to RXF; 161, the last word, loops.
    for rnd in range(2):                                  # the second load rewrites every word
        vals = [rng.getrandbits(16) for _ in range(R.IMEM_WORDS - 4)]
        a = Asm()
        for x in vals:
            a.xor(1, x)
        a.fput(1, I.FIFO_RX).shr(1, 8).fput(1, I.FIFO_RX)
        a.L("e").jmp("e")
        assert len(a.words()) == R.IMEM_WORDS
        exp = 0
        for x in vals:
            exp ^= x
        chip = Chip(dut)
        await chip.reset()
        await chip.bus.load_program(a.words())
        await chip.bus.start()
        await ClockCycles(dut.clk, R.IMEM_WORDS + 40)
        got = await chip.bus.read(R.RXF) | (await chip.bus.read(R.RXF) << 8)
        assert got == exp, f"IMEM round {rnd}: result {got:#06x}, expected {exp:#06x}"
        await chip.bus.write(R.RUN, 0)

    # DMEM: the host fills all 256 bytes; the firmware inverts each one and reads it back in the very
    # next instruction (st then ld of the same byte); it counts mismatches and sends the count to RXF.
    chip = Chip(dut)
    await chip.reset()
    a = Asm().mov(1, 0).mov(3, 0)                         # r1 address, r3 mismatches
    a.L("l").ld(0, 1).xor(0, 0xFF).st(0, 1).ld(2, 1)      # invert, write, read straight back
    a.cmp(2, I.R(0)).jmp("ok", I.Z).add(3, 1)
    a.L("ok").add(1, 1).cmp(1, 256).jmp("l", I.NZ)        # bytes 0 .. 255
    a.fput(3, I.FIFO_RX).host_irq()
    idle(a)
    await chip.bus.load_program(a.words())
    data = [rng.getrandbits(8) for _ in range(256)]
    data[0], data[255] = 0x00, 0xFF                       # the first and last byte, both extremes
    await chip.bus.dmem_write(0, data)
    assert await chip.bus.dmem_read(0, 256) == data, "host write and read back of all 256 bytes"
    await chip.bus.start()
    # The host reads byte 0 while the firmware runs: the host wins the port and the firmware retries.
    for _ in range(20):
        b0 = (await chip.bus.dmem_read(0, 1))[0]
        assert b0 in (0x00, 0xFF), f"byte 0 read during the run: {b0:#04x}"
    for _ in range(200):
        if await chip.bus.read(R.STATUS2) & R.S2_IRQ:
            break
    else:
        raise AssertionError("the DMEM firmware never finished")
    assert await chip.bus.read(R.RXF) == 0, "a byte read straight after its write was wrong"
    assert await chip.bus.dmem_read(0, 256) == [x ^ 0xFF for x in data], "DMEM after the firmware pass"
    await chip.bus.write(R.RUN, 0)
