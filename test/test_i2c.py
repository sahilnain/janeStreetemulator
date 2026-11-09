# SPDX-License-Identifier: Apache-2.0
"""
test_i2c.py: top-level I2C tests (v2), run through test/tb.v.

What this file does
-------------------
SCL = P6, SDA = P7, both open drain (PIN_OD), wired-AND with the
testbench devices in i2c.py.
Chip as master (pe_fw.i2c_master, DMEM-driven): an i2c.I2cSlave register
device at 0x50 answers. The host writes the request into DMEM, rings
DOORBELL, waits for the controller IRQ, reads the status and data.
Chip as slave (pe_fw.i2c_slave at 0x42, DMEM = its registers): the
testbench is the master (i2c.I2cMaster).

Tests
-----
1. test_master_write_read   B = 126 (375 kHz): write pointer + 3 bytes, then pointer
                            write + repeated START + read 3 (ACK, ACK, NACK).
                            Bus checks: START/STOP counts, SDA never
                            changes in the same cycle SCL falls (hold >= 1), and every
                            limit of the I2C standard for this speed.
2. test_master_speeds       each I2C speed at its fastest B, with the SCL period B + 2 (blocks.md
                            3.3, STRETCH): 100 kHz, 375 kHz, 960 kHz, with all limits checked.
3. test_master_addr_nack    no device at the address -> status 1, STOP sent.
4. test_master_data_nack    the device NACKs the 2nd data byte -> status 2.
5. test_master_stretch      the device stretches SCL after the address ACK
                            and before every read byte: data still correct.
6. test_slave_write         master writes pointer + 4 bytes -> chip DMEM.
7. test_slave_read          master reads 4 bytes from a pointer (repeated
                            START); the last byte is NACKed.
8. test_slave_wrong_addr    another address is NACKed and ignored; the next
                            transfer to the chip works.
9. test_timing_check        error on purpose: B = 125 and too fast a speed for a mode are
                            reported by the checker, so a pass above means something.
"""

import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "common"))

import cocotb  # noqa: E402
import pe_build  # noqa: E402
import i2c  # noqa: E402
import pe_fw  # noqa: E402
import pe_host as R  # noqa: E402
from chip import Chip  # noqa: E402
from cocotb.triggers import ClockCycles  # noqa: E402

SCL, SDA = 6, 7


async def master_setup(dut, B=126, **dev_kw):
    chip = Chip(dut)
    await chip.reset()
    await chip.bus.write(R.PIN_OD, 0xC0)
    await chip.bus.load_program(pe_fw.i2c_master(B=B, scl=SCL, sda=SDA).words())
    await chip.bus.owners(R.owner_map(p6=R.OWN_TX, p7=R.OWN_RX))
    await chip.bus.start()
    dev = i2c.I2cSlave(SCL, SDA, 0x50, **dev_kw)
    chip.devices.append(dev)
    await ClockCycles(dut.clk, 30)
    return chip, dev


async def request(chip, addr, wdata=(), nread=0, limit=20000):
    bus = chip.bus
    await bus.dmem_write(0, [addr, len(wdata), nread] + list(wdata))
    await bus.write(R.DOORBELL, 0)
    for _ in range(limit):
        if await bus.read(R.STATUS2) & R.S2_IRQ:
            break
    else:
        pc, cs = await bus.read16(R.PC), await bus.read(R.CSTAT)
        tr_scl, tr_sda = chip.trace(SCL), chip.trace(SDA)
        raise AssertionError(f"no controller IRQ: pc={pc} cstat={cs:#x} SCL edges="
                             f"{sum(1 for x, y in zip(tr_scl, tr_scl[1:]) if x != y)} "
                             f"SDA edges={sum(1 for x, y in zip(tr_sda, tr_sda[1:]) if x != y)} "
                             f"now SCL={tr_scl[-1]} SDA={tr_sda[-1]}")
    await bus.write(R.STATUS2, R.S2_IRQ)
    status = (await bus.dmem_read(0x7F, 1))[0]
    return status, (await bus.dmem_read(0x80, nread) if nread else [])


async def write_read(chip, dev, ptr=0x20):
    data = [random.getrandbits(8) for _ in range(3)]
    st, _ = await request(chip, 0x50, [ptr] + data)
    assert st == 0
    assert dev.mem[ptr:ptr + 3] == data
    st, got = await request(chip, 0x50, [ptr], nread=3)
    assert st == 0 and got == data, got
    return data


@pe_build.test("i2c")
async def test_master_write_read(dut):
    chip, dev = await master_setup(dut)
    await write_read(chip, dev)
    assert not (v := i2c.check_timing(chip.samples, SCL, SDA, "FM")), v
    assert dev.starts == 3 and dev.stops == 2, (dev.starts, dev.stops)   # 2nd: START + rep. START
    assert dev.min_hold is not None and dev.min_hold >= 1, f"SDA hold {dev.min_hold}"
    assert [k for k, _ in dev.log] == ["w"] * 4 + ["w"] + ["r"] * 3
    assert await chip.bus.read(R.STATUS) & 0xF0 == 0


@pe_build.test("i2c")
async def test_master_speeds(dut):
    for mode, B in (("SM", 478), ("FM", 126), ("FM+", 48)):
        chip, dev = await master_setup(dut, B=B)
        await write_read(chip, dev, ptr=B & 0xFF)
        await chip.bus.write(R.RUN, 0)
        assert not (v := i2c.check_timing(chip.samples, SCL, SDA, mode)), f"{mode}: {v}"


@pe_build.test("i2c")
async def test_master_addr_nack(dut):
    chip, dev = await master_setup(dut)
    st, _ = await request(chip, 0x51, [1, 2])
    assert st == 1
    assert dev.stops == 1 and dev.log == []


@pe_build.test("i2c")
async def test_master_data_nack(dut):
    chip, dev = await master_setup(dut, nack_after=2)        # pointer + 1 data byte
    st, _ = await request(chip, 0x50, [0x10, 0xAA, 0xBB, 0xCC])
    assert st == 2
    assert dev.mem[0x10] == 0xAA and dev.mem[0x11] == 0
    assert dev.stops == 1


@pe_build.test("i2c")
async def test_master_stretch(dut):
    chip, dev = await master_setup(dut, stretch={"addr": 300, "read": 150})
    await write_read(chip, dev, ptr=0x40)


async def slave_setup(dut, h=60):
    chip = Chip(dut)
    await chip.reset()
    await chip.bus.write(R.PIN_OD, 0xC0)
    await chip.bus.load_program(pe_fw.i2c_slave(0x42, SCL, SDA).words())
    await chip.bus.owners(R.owner_map(p7=R.OWN_TX))
    await chip.bus.start()
    m = i2c.I2cMaster(chip, SCL, SDA, h=h)
    chip.devices.append(m)
    await ClockCycles(dut.clk, 30)
    return chip, m


@pe_build.test("i2c")
async def test_slave_write(dut):
    chip, m = await slave_setup(dut)
    data = [random.getrandbits(8) for _ in range(4)]
    acks = await m.write(0x42, [0x10] + data)
    assert acks == [True] * 6, acks
    await ClockCycles(dut.clk, 50)
    await chip.bus.write(R.RUN, 0)
    assert await chip.bus.dmem_read(0x10, 4) == data


@pe_build.test("i2c")
async def test_slave_read(dut):
    chip, m = await slave_setup(dut)
    data = [random.getrandbits(8) for _ in range(4)]
    await chip.bus.dmem_write(0x30, data)
    acks, got = await m.read(0x42, 4, ptr=0x30)
    assert acks == [True, True, True], acks
    assert got == data, got
    # the chip drives SDA here, and the testbench SCL: only the data setup is the chip's
    assert not (v := i2c.check_timing(chip.samples, SCL, SDA, "SM", only=("su_dat",))), v


@pe_build.test("i2c")
async def test_slave_wrong_addr(dut):
    chip, m = await slave_setup(dut)
    acks = await m.write(0x43, [0x00, 0x11])
    assert acks[0] is False
    acks = await m.write(0x42, [0x05, 0x99])
    assert acks == [True, True, True]
    await ClockCycles(dut.clk, 50)
    await chip.bus.write(R.RUN, 0)
    assert await chip.bus.dmem_read(0x05, 1) == [0x99]


@pe_build.test("i2c")
async def test_timing_check(dut):
    for mode, B, name in (("FM", 125, "low"), ("SM", 400, "fscl"), ("FM+", 40, "low")):
        chip, dev = await master_setup(dut, B=B)
        await write_read(chip, dev, ptr=0x20)
        await chip.bus.write(R.RUN, 0)
        v = i2c.check_timing(chip.samples, SCL, SDA, mode)
        assert any(x.startswith(name) for x in v), f"{mode} B={B} should break {name}: {v}"
