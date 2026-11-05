# SPDX-License-Identifier: Apache-2.0
"""
test_spi.py: top-level SPI tests (v2), run through test/tb.v.

What this file does
-------------------
Chip as SPI master (pe_fw.spi_master, DMEM-driven): a cycle-level SPI slave
model (spi.SpiSlave) answers on MISO. The host writes the request into
DMEM, rings DOORBELL, waits for the controller IRQ and reads the reply
from DMEM. Chip as SPI slave (pe_fw.spi_slave): the testbench is the
master (spi.master_xfer). Pins: SCK P2, MOSI P3, MISO P4, CS P5.

Tests
-----
1. test_master_modes        modes 0-3, MSB-first, B = 2 (SCK = 24 MHz):
                            data both ways, 8 SCK pulses per byte, every
                            SCK half period is exactly 1 cycle inside a
                            byte, CS low for the whole transfer.
2. test_master_lsb_slow     LSB-first at B = 10 (4.8 MHz) and odd B = 3.
3. test_master_long         64 bytes at 24 MHz: data, and the gap between
                            bytes (controller loop) is bounded.
4. test_master_sdly         at B = 4 the MISO sample delay works for SDLY
                            2..4 and fails on purpose for SDLY = 0 (data
                            sampled before it arrives).
5. test_slave_mode0         chip as slave, mode 0, 6 bytes each way.
6. test_slave_mode2_lsb     mode 2 (CPOL 1), LSB-first.
7. test_slave_two_frames    two CS frames back to back.
8. test_slave_fast          SCK half period 6 cycles (4 MHz).
9. test_slave_underrun      TX FIFO empty at select -> STATUS2.tx_underrun
                            (error on purpose); the received data is
                            still correct.
10. test_slave_all_modes    modes 0-3 x MSB / LSB: data both ways, and MISO
                            changes only after shift edges.
11. test_slave_max_rate     fastest SCK per mode: half period 6 (CPHA 0, 4
                            MHz) and 4 (CPHA 1, 6 MHz); two frames per mode.
12. test_slave_cpha1_underrun  CPHA 1 with an empty TX FIFO: underrun flagged,
                            MOSI data still received.
"""

import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "common"))

import cocotb  # noqa: E402
import pe_fw  # noqa: E402
import pe_host as R  # noqa: E402
import spi  # noqa: E402
from chip import Chip  # noqa: E402
from cocotb.triggers import ClockCycles  # noqa: E402

SCK, MOSI, MISO, CS = 2, 3, 4, 5


async def master_setup(dut, **kw):
    chip = Chip(dut)
    await chip.reset()
    await chip.bus.load_program(pe_fw.spi_master(**kw).words())
    await chip.bus.owners(R.owner_map(p2=R.OWN_TX, p3=R.OWN_TX))
    await chip.bus.start()
    await ClockCycles(dut.clk, 30)
    return chip


async def master_xfer(chip, data):
    bus = chip.bus
    await bus.dmem_write(0, [len(data)] + list(data))
    await bus.write(R.DOORBELL, 0)
    for _ in range(2000):
        if await bus.read(R.STATUS2) & R.S2_IRQ:
            break
    else:
        raise AssertionError("no controller IRQ")
    await bus.write(R.STATUS2, R.S2_IRQ)
    return await bus.dmem_read(0x80, len(data))


@cocotb.test()
async def test_master_modes(dut):
    for mode in range(4):
        cpol, cpha = mode >> 1, mode & 1
        chip = await master_setup(dut, B=2, cpol=cpol, cpha=cpha, msb=1)
        data = [random.getrandbits(8) for _ in range(6)]
        reply = [random.getrandbits(8) for _ in range(6)]
        dev = spi.SpiSlave(SCK, MOSI, MISO, CS, cpol, cpha, 1, tx=reply)
        chip.devices.append(dev)
        got = await master_xfer(chip, data)
        assert dev.rx == data, f"mode {mode}: slave got {dev.rx}"
        assert got == reply, f"mode {mode}: master got {got}"
        assert len(dev.edges) == 16 * len(data), len(dev.edges)
        for b in range(len(data)):
            e = dev.edges[16 * b:16 * b + 16]
            assert all(y - x == 1 for x, y in zip(e, e[1:])), f"24 MHz inside byte {b}: {e}"
        assert set(chip.trace(SCK)[-20:]) == {cpol}, "SCK idles at CPOL"
        assert chip.trace(CS)[-1] == 1
        await chip.bus.write(R.RUN, 0)


@cocotb.test()
async def test_master_lsb_slow(dut):
    for B in (10, 3):
        chip = await master_setup(dut, B=B, cpol=0, cpha=0, msb=0)
        data = [random.getrandbits(8) for _ in range(4)]
        reply = [random.getrandbits(8) for _ in range(4)]
        dev = spi.SpiSlave(SCK, MOSI, MISO, CS, 0, 0, 0, tx=reply)
        chip.devices.append(dev)
        assert await master_xfer(chip, data) == reply, B
        assert dev.rx == data, B
        e = dev.edges[:16]
        assert e[2] - e[0] == B, f"SCK period {e[2] - e[0]} != B = {B}"
        await chip.bus.write(R.RUN, 0)


@cocotb.test()
async def test_master_long(dut):
    chip = await master_setup(dut, B=2)
    data = [random.getrandbits(8) for _ in range(64)]
    reply = [random.getrandbits(8) for _ in range(64)]
    dev = spi.SpiSlave(SCK, MOSI, MISO, CS, tx=reply)
    chip.devices.append(dev)
    assert await master_xfer(chip, data) == reply
    assert dev.rx == data
    gaps = [dev.edges[16 * b] - dev.edges[16 * b - 1] for b in range(1, 64)]
    assert max(gaps) <= 20, f"inter-byte gap {max(gaps)} cycles"
    dut._log.info(f"SPI 24 MHz: 64 bytes, inter-byte gap {min(gaps)}..{max(gaps)} cycles, "
                  f"{dev.edges[-1] - dev.edges[0]} cycles first to last edge")


@cocotb.test()
async def test_master_sdly(dut):
    for sdly, ok in ((2, True), (3, True), (4, True), (0, False)):
        chip = await master_setup(dut, B=4, sdly=sdly)
        reply = [0x5A, 0xC3, 0x0F, 0xF0]
        chip.devices.append(spi.SpiSlave(SCK, MOSI, MISO, CS, tx=list(reply)))
        got = await master_xfer(chip, [0] * 4)
        assert (got == reply) == ok, f"SDLY {sdly}: {got}"
        await chip.bus.write(R.RUN, 0)


async def slave_setup(dut, txf, **kw):
    chip = Chip(dut)
    await chip.reset()
    chip.drive.update({SCK: kw.get("cpol", 0), MOSI: 0, CS: 1})
    await chip.bus.load_program(pe_fw.spi_slave(**kw).words())
    await chip.bus.owners(R.owner_map(p4=R.OWN_TX))
    await chip.bus.set_addr(R.TXF)
    for d in txf:
        await chip.bus.write_same(d)
    await chip.bus.start()
    await ClockCycles(dut.clk, 30)
    return chip


@cocotb.test()
async def test_slave_mode0(dut):
    resp = [random.getrandbits(8) for _ in range(6)]
    chip = await slave_setup(dut, resp + [0xEE])      # one extra byte: see pe_fw.spi_slave
    data = [random.getrandbits(8) for _ in range(6)]
    got = await spi.master_xfer(chip, data, h=8)
    assert got == resp, f"MISO {got}"
    assert await chip.read_rx(6) == data
    assert not await chip.bus.read(R.STATUS2) & R.S2_TX_UNDER


@cocotb.test()
async def test_slave_mode2_lsb(dut):
    resp = [random.getrandbits(8) for _ in range(4)]
    chip = await slave_setup(dut, resp + [0xEE], cpol=1, msb=0)
    data = [random.getrandbits(8) for _ in range(4)]
    got = await spi.master_xfer(chip, data, h=8, cpol=1, msb=0)
    assert got == resp
    assert await chip.read_rx(4) == data


@cocotb.test()
async def test_slave_two_frames(dut):
    r1, r2 = [0x11, 0x22], [0x33, 0x44, 0x55]
    chip = await slave_setup(dut, r1 + [0xE1] + r2 + [0xE2])
    d1, d2 = [0xA1, 0xA2], [0xB1, 0xB2, 0xB3]
    assert await spi.master_xfer(chip, d1, h=8) == r1
    await ClockCycles(dut.clk, 40)
    assert await spi.master_xfer(chip, d2, h=8) == r2
    assert await chip.read_rx(5) == d1 + d2


@cocotb.test()
async def test_slave_fast(dut):
    resp = [random.getrandbits(8) for _ in range(5)]
    chip = await slave_setup(dut, resp + [0xEE])
    data = [random.getrandbits(8) for _ in range(5)]
    assert await spi.master_xfer(chip, data, h=6) == resp
    assert await chip.read_rx(5) == data


@cocotb.test()
async def test_slave_underrun(dut):
    chip = await slave_setup(dut, [])
    data = [0x12, 0x34]
    await spi.master_xfer(chip, data, h=8)
    assert await chip.bus.read(R.STATUS2) & R.S2_TX_UNDER
    assert await chip.read_rx(2) == data


@cocotb.test()
async def test_slave_all_modes(dut):
    for mode in range(4):
        for msb in (1, 0):
            cpol, cpha = mode >> 1, mode & 1
            resp = [random.getrandbits(8) for _ in range(5)]
            chip = await slave_setup(dut, resp + [0xEE], cpol=cpol, cpha=cpha, msb=msb)
            data = [random.getrandbits(8) for _ in range(5)]
            t0 = len(chip.samples)
            got = await spi.master_xfer(chip, data, h=8, cpol=cpol, cpha=cpha, msb=msb)
            assert got == resp, f"mode {mode} msb {msb}: MISO {got} != {resp}"
            assert await chip.read_rx(5) == data, f"mode {mode} msb {msb}"
            spi.check_miso_edges(chip.samples[t0:], SCK, MISO, CS, cpol, cpha)
            assert await chip.bus.read(R.STATUS2) == 0
            await chip.bus.write(R.RUN, 0)


@cocotb.test()
async def test_slave_max_rate(dut):
    for mode in range(4):
        cpol, cpha = mode >> 1, mode & 1
        h = 4 if cpha else 6
        r1, r2 = [random.getrandbits(8) for _ in range(4)], [random.getrandbits(8) for _ in range(3)]
        chip = await slave_setup(dut, r1 + [0xE1] + r2 + [0xE2], cpol=cpol, cpha=cpha)
        d1, d2 = [random.getrandbits(8) for _ in range(4)], [random.getrandbits(8) for _ in range(3)]
        assert await spi.master_xfer(chip, d1, h=h, cpol=cpol, cpha=cpha) == r1, f"mode {mode}"
        await ClockCycles(dut.clk, 40)
        assert await spi.master_xfer(chip, d2, h=h, cpol=cpol, cpha=cpha) == r2, f"mode {mode}"
        assert await chip.read_rx(7) == d1 + d2, f"mode {mode}"
        await chip.bus.write(R.RUN, 0)


@cocotb.test()
async def test_slave_cpha1_underrun(dut):
    chip = await slave_setup(dut, [], cpha=1)
    data = [0x12, 0x34]
    await spi.master_xfer(chip, data, h=8, cpha=1)
    assert await chip.bus.read(R.STATUS2) & R.S2_TX_UNDER
    assert await chip.read_rx(2) == data
