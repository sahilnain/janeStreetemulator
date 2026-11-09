# SPDX-License-Identifier: Apache-2.0
"""
test.py: top-level tests for tt_um_sahil_protoemu v2 (run through test/tb.v).

What this file does
-------------------
Tests the whole chip only through its pins, the way the RP2040 host will
use it: the 4-bit host bus on ui_in/uo_out (pe_host.HostBus) loads
firmware, configures pins and moves bytes through the FIFOs and data
memory. The protocol pins are uio. The same tests run for RTL and for gate
level (make GATES=yes). Protocol tests: test_uart.py, test_spi.py,
test_i2c.py; two emulators: test_dual.py (make TB=tb_dual COCOTB_TEST_MODULES=test_dual).

Tests
-----
1. test_registers      reset state (no pin drives), ID/VERSION, RW readback
                       of every RW register, unmapped address reads 0.
2. test_imem_err_irq   an IMEM commit while running is rejected: imem_err +
                       IRQ pin; ERR_CLR clears them.
3. test_open_drain     open-drain pin: uio_out stays 0, uio_oe toggles.
4. test_owner_err      controller write to a pin owned by a BUSY unit is
                       dropped and sets owner_err; to an idle unit's pin it
                       is allowed.
5. test_ill_stk_err    reserved opcode -> ill_err; ret on empty stack and
                       call on a full stack -> stk_err; execution goes on.
6. test_call_ret_alu   call/ret nesting, ALU flags and djnz loops; the
                       result is written to DMEM and read by the host.
7. test_doorbell_irq   host DOORBELL -> DB flag -> firmware answers with
                       `host irq`; IRQ pin follows IRQ_EN[3]; W1C clears.
8. test_dmem_host      host DMEM block write/read with auto-increment while
                       the controller runs ld/st on the same memory.
9. test_fifo_errors    TX FIFO overrun from the host; RX FIFO read when
                       empty; ERR_CLR.
10. test_restart       RESTART while a unit is busy stops it; the program
                       starts again from START_PC.
11. test_wait_timeout  wait with a timeout sets TO; wait success clears it.
"""

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


@cocotb.test()
async def test_registers(dut):
    chip = Chip(dut)
    await chip.reset()
    bus = chip.bus
    assert int(dut.uio_oe.value) == 0, "no protocol pin may drive after reset"
    assert (int(dut.uo_out.value) >> 6) == 0
    assert await bus.read(R.ID) == 0x50
    assert await bus.read(R.VERSION) == 0x02
    assert await bus.read(R.STATUS) == R.ST_TXF_EMPTY | R.ST_RXF_EMPTY
    assert await bus.read(R.STATUS2) == 0
    assert await bus.read(R.CSTAT) == 0
    # PIN_OD is part of I2C: without it, the register ignores writes (blocks.md Section 13)
    od = 0xA5 if pe_build.has("i2c") else 0x00
    for a, x, back in ((R.PIN_OD, 0xA5, od), (R.IRQ_EN, 0x0A, 0x0A), (R.START_PC, 0x2A, 0x2A), (R.START_PC + 1, 0x00, 0x00),
                       (R.PIN_OWNER, 0x96, 0x96), (R.PIN_OWNER + 1, 0x5A, 0x5A), (R.IMEM_ADDR, 0x3C, 0x3C),
                       (R.DMEM_ADDR, 0xC3, 0xC3), (R.RUN, 0x01, 0x01), (R.RUN, 0x00, 0x00)):
        await bus.write(a, x)
        assert await bus.read(a) == back, hex(a)
    await bus.write(R.PIN_OD, 0)
    await bus.write(R.IRQ_EN, 0)
    await bus.owners(0)
    for a in (0x7F, 0x30, 0x1C, 0xFF):
        assert await bus.read(a) == 0


@cocotb.test()
async def test_imem_err_irq(dut):
    chip = Chip(dut)
    await chip.reset()
    bus = chip.bus
    await bus.load_program(Asm().jmp(0).words())
    await bus.start()
    await bus.write(R.IRQ_EN, 0x4)
    assert bus.irq() == 0
    await bus.load_program(Asm().nop().words(), start=0)  # rejected: running
    assert await bus.read(R.STATUS) & R.ST_IMEM_ERR
    assert bus.irq() == 1, "IRQ pin"
    await bus.write(R.ERR_CLR, 0x80)
    assert not await bus.read(R.STATUS) & R.ST_IMEM_ERR
    assert bus.irq() == 0
    await bus.write(R.RUN, 0)


@pe_build.test("i2c")
async def test_open_drain(dut):
    chip = Chip(dut)
    await chip.reset()
    bus = chip.bus
    a = Asm().L("l").set(2, I.SRC_0, dly=7).set(2, I.SRC_1, dly=7).jmp("l")
    await bus.load_program(a.words())
    await bus.write(R.PIN_OD, 0x04)
    await bus.start()
    oes = []
    for _ in range(200):
        await FallingEdge(dut.clk)
        assert (int(dut.uio_out.value) >> 2) & 1 == 0, "open-drain pin must never drive high"
        oes.append((int(dut.uio_oe.value) >> 2) & 1)
    assert 0 in oes and 1 in oes, "oe must toggle"
    await bus.write(R.RUN, 0)


@cocotb.test()
async def test_owner_err(dut):
    chip = Chip(dut)
    await chip.reset()
    bus = chip.bus
    a = Asm()
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(3)).ucfg(I.TX, I.TX_BIT, 50)
    a.setoe(3, 1).set(3, I.SRC_0)            # TX idle: controller may drive P3
    a.ustart(I.M_TX, 0xFF)
    a.set(3, I.SRC_0)                        # TX busy: dropped, owner_err
    a.uwait(I.M_TX)
    a.L("e").jmp("e")
    await bus.load_program(a.words())
    await bus.owners(R.owner_map(p3=R.OWN_TX))
    await bus.start()
    await ClockCycles(dut.clk, 600)
    tr = chip.trace(3)
    i0 = tr.index(0)                         # controller drove P3 low (TX idle)
    i1 = tr.index(1, i0)                     # packet 0xFF starts
    assert 0 not in tr[i1:], "the set during the packet must be dropped"
    assert await bus.read(R.STATUS) & R.ST_OWNER_ERR
    await bus.write(R.ERR_CLR, 0x40)
    assert not await bus.read(R.STATUS) & R.ST_OWNER_ERR
    await bus.write(R.RUN, 0)


@cocotb.test()
async def test_ill_stk_err(dut):
    chip = Chip(dut)
    await chip.reset()
    bus = chip.bus
    a = Asm()
    a.raw(I.word(5)).raw(I.word(40)).raw(I.word(63))    # reserved opcodes: nop + ill_err
    a.mov(1, 0x11)
    a.ret()                                             # empty stack: stk_err, continue
    a.mov(2, 0x22)
    a.call("f1")
    a.mov(0, 0).st(1, 0, inc=1).st(2, 0, inc=1).st(3, 0, inc=1)
    a.L("e").jmp("e")
    for n in range(1, 6):                               # call depth 5 > STACK_DEPTH 4
        a.L(f"f{n}")
        if n < 5:
            a.call(f"f{n + 1}")
        else:
            a.mov(3, 0x33)
        a.ret()
    await bus.load_program(a.words())
    await bus.start()
    await ClockCycles(dut.clk, 200)
    s2 = await bus.read(R.STATUS2)
    assert s2 & R.S2_ILL and s2 & R.S2_STK, hex(s2)
    await bus.write(R.RUN, 0)
    # the 5th call ran as nop (stack full): f5 never ran, so r3 stayed 0, and
    # the returns still came back to the main program
    assert await bus.dmem_read(0, 3) == [0x11, 0x22, 0x00]
    await bus.write(R.STATUS2, R.S2_ILL | R.S2_STK)
    assert await bus.read(R.STATUS2) == 0


@cocotb.test()
async def test_call_ret_alu(dut):
    chip = Chip(dut)
    await chip.reset()
    bus = chip.bus
    a = Asm()
    a.mov(6, 0x40)                       # dmem pointer
    a.mov(0, 0xFFFF).add(0, 1)           # 0, Z=1 C=1
    a.call("flags")
    a.mov(0, 5).sub(0, 7)                # 0xFFFE, Z=0 C=1 (borrow)
    a.call("flags")
    a.st(0, 6, inc=1)                    # low byte 0xFE
    a.mov(0, 0x8001).shl(0, 1)           # 0x0002, C=1
    a.call("flags")
    a.st(0, 6, inc=1)
    a.mov(0, 0x0003).shr(0, 2)           # 0x0000, C=1 (bit 1 out), Z=1
    a.call("flags")
    a.mov(0, 0xF0F0).and_(0, 0x0FF0).or_(0, 0x0001).xor(0, 0x00FF)   # 0x000E
    a.st(0, 6, inc=1)
    a.mov(1, 0x1234).mov(2, I.R(1)).cmp(2, 0x1234)                      # Z
    a.call("flags")
    a.mov(3, 10).mov(4, 0)
    a.L("lp").add(4, I.R(3)).djnz(3, "lp")   # 10+9+..+1 = 55
    a.st(4, 6, inc=1)
    a.mov(5, 0x80).bitin(5, 9, msb=1)    # pin 9 does not exist: reads 0 -> 0x100
    a.shr(5, 4)
    a.st(5, 6, inc=1)                    # 0x10
    a.L("e").jmp("e")
    a.L("flags")                         # store {Z, C} as a byte: 2*Z + C
    a.mov(7, 0)
    a.jmp("nz", I.NZ).or_(7, 2).L("nz")
    a.jmp("nc", I.NC).or_(7, 1).L("nc")
    a.st(7, 6, inc=1)
    a.ret()
    await bus.load_program(a.words())
    await bus.start()
    await ClockCycles(dut.clk, 600)
    await bus.write(R.RUN, 0)
    got = await bus.dmem_read(0x40, 10)
    # flags(0+carry)=3, flags(sub)=1, FE, flags(shl)=1, 02, flags(shr)=3, 0E, flags(cmp)=2, 55, 0x10
    # note: or_ in `flags` clobbers Z/C only after they are tested
    assert got == [3, 1, 0xFE, 1, 0x02, 3, 0x0E, 2, 55, 0x10], repr(got)


@cocotb.test()
async def test_doorbell_irq(dut):
    chip = Chip(dut)
    await chip.reset()
    bus = chip.bus
    a = Asm()
    a.L("w").jmp("w", I.NDB)
    a.dbclr()
    a.host_irq()
    a.jmp("w")
    await bus.load_program(a.words())
    await bus.start()
    await bus.write(R.IRQ_EN, 0x08)
    await ClockCycles(dut.clk, 50)
    assert bus.irq() == 0
    assert not await bus.read(R.CSTAT) & R.CS_DB
    await bus.write(R.DOORBELL, 0)
    await ClockCycles(dut.clk, 20)
    assert await bus.read(R.STATUS2) & R.S2_IRQ
    assert bus.irq() == 1
    assert not await bus.read(R.CSTAT) & R.CS_DB, "firmware cleared DB"
    await bus.write(R.STATUS2, R.S2_IRQ)
    assert bus.irq() == 0
    await bus.write(R.IRQ_EN, 0)
    await bus.write(R.RUN, 0)


@cocotb.test()
async def test_dmem_host(dut):
    chip = Chip(dut)
    await chip.reset()
    bus = chip.bus
    # firmware: copy dmem[0..15] -> dmem[0x80..0x8F] + 1, forever
    a = Asm()
    a.L("top").mov(0, 0).mov(1, 0x80).mov(3, 16)
    a.L("lp").ld(2, 0, inc=1).add(2, 1).st(2, 1, inc=1).djnz(3, "lp")
    a.jmp("top")
    data = list(range(0x30, 0x40))
    await bus.dmem_write(0, data)
    await bus.load_program(a.words())
    await bus.start()
    # host traffic while the controller runs (arbitration)
    for _ in range(3):
        assert await bus.dmem_read(0, 16) == data
    await ClockCycles(dut.clk, 300)
    assert await bus.dmem_read(0x80, 16) == [d + 1 for d in data]
    await bus.write(R.RUN, 0)
    # wrap of the auto-incremented address
    await bus.dmem_write(0xFF, [0xAB, 0xCD])
    assert await bus.dmem_read(0xFF, 2) == [0xAB, 0xCD]
    assert await bus.read(R.DMEM_ADDR) == 0x01


@cocotb.test()
async def test_fifo_errors(dut):
    chip = Chip(dut)
    await chip.reset()
    bus = chip.bus
    await bus.read(R.RXF)                            # empty read: no state change
    assert await bus.read(R.STATUS) & R.ST_RXF_EMPTY
    await bus.set_addr(R.TXF)
    for d in range(9):                               # depth 8: the 9th is dropped
        await bus.write_same(d)
    st = await bus.read(R.STATUS)
    assert st & R.ST_TXF_FULL and st & R.ST_TXF_OVR
    assert await bus.read(R.FIFO_CNT) == 0x08
    await bus.write(R.IRQ_EN, 0x04)
    assert bus.irq() == 1
    await bus.write(R.ERR_CLR, 0x10)
    assert not await bus.read(R.STATUS) & R.ST_TXF_OVR
    assert bus.irq() == 0
    # firmware drains TXF into RXF: the order is kept, 8 bytes
    a = Asm().L("l").fget(0, I.FIFO_TX).fput(0, I.FIFO_RX).jmp("l")
    await bus.load_program(a.words())
    await bus.start()
    await ClockCycles(dut.clk, 100)
    assert await bus.read(R.FIFO_CNT) == 0x80
    got = [await bus.read(R.RXF) for _ in range(8)]
    assert got == list(range(8))
    await bus.write(R.IRQ_EN, 0)
    await bus.write(R.RUN, 0)


@cocotb.test()
async def test_restart(dut):
    chip = Chip(dut)
    await chip.reset()
    bus = chip.bus
    a = Asm()
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(0)).ucfg(I.TX, I.TX_BIT, 1000)
    a.set(0, I.SRC_1).setoe(0, 1)
    a.ustart(I.M_TX, 0x00)               # long packet of zeros
    a.L("e").jmp("e")
    await bus.owners(R.owner_map(p0=R.OWN_TX))
    await bus.load_program(a.words())
    await bus.start()
    await ClockCycles(dut.clk, 300)
    assert int(dut.uio_out.value) & 1 == 0, "packet running"
    await bus.write(R.RUN, 0)
    await bus.write(R.RESTART, 1)        # aborts the TX unit
    await ClockCycles(dut.clk, 5)
    # the TX unit is idle: the controller can drive P0 again
    b = Asm().set(0, I.SRC_1).L("e").jmp("e")
    await bus.load_program(b.words())
    await bus.start()
    await ClockCycles(dut.clk, 20)
    assert int(dut.uio_out.value) & 1 == 1
    assert not await bus.read(R.STATUS) & R.ST_OWNER_ERR
    assert await bus.read16(R.PC) == 1
    await bus.write(R.RUN, 0)


@cocotb.test()
async def test_wait_timeout(dut):
    chip = Chip(dut)
    await chip.reset()
    bus = chip.bus
    chip.ext = 0xFF ^ (1 << 4)           # P4 held low
    a = Asm()
    a.mov(1, 20).wait(4, 1, rd=1)        # times out: TO = 1
    a.jmp("t1", I.TO).mov(0, 0xEE).L("t1")
    a.mov(6, 0).st(1, 6, inc=1)          # r1 counted down to 0
    a.wait(5, 1, rd=1)                   # P5 high: immediate success, TO = 0 (r1 = 0 left)
    a.jmp("t2", I.NTO).mov(0, 0xEE).L("t2")
    a.mov(0, 0x5A).st(0, 6, inc=1)
    a.L("e").jmp("e")
    await bus.load_program(a.words())
    await bus.start()
    await ClockCycles(dut.clk, 200)
    await bus.write(R.RUN, 0)
    assert await bus.dmem_read(0, 2) == [0, 0x5A]
