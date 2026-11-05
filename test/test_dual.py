# SPDX-License-Identifier: Apache-2.0
"""
test_dual.py: two emulators talking to each other (run: make TB=tb_dual
COCOTB_TEST_MODULES=test_dual).

What this file does
-------------------
tb_dual.v holds emulator 1 (e1_*) and emulator 2 (e2_*) on one clock. The
pads coroutine crosses the UART lines: e1.P0 (TX) -> e2.P1 (RX) and
e2.P0 (TX) -> e1.P1 (RX). Each emulator has its own host bus, driven
concurrently. Both run pe_fw.uart.

Tests
-----
1. test_e1_to_e2          emulator 1 sends, emulator 2 receives (B = 112, 24).
2. test_both_directions   both send at the same time, different data.
3. test_baud_mismatch     e1 at B = 111, e2 at B = 113 (1.8% apart), both ways.
4. test_spi_all_modes     e1 SPI master, e2 SPI slave, modes 0-3 x MSB / LSB:
                          SCK, MOSI, CS e1 -> e2; MISO e2 -> e1.
5. test_usb_both_speeds   e1 USB host, e2 USB device on one shared bus, low and full
                          speed: SETUP, OUT, IN, NAK; usb.py checks every packet
                          on the wire and fails on bus contention.
"""

import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "common"))

import cocotb  # noqa: E402
import pe_fw  # noqa: E402
import pe_host as R  # noqa: E402
import uart  # noqa: E402
import usb  # noqa: E402
from chip import Chip  # noqa: E402
from cocotb.clock import Clock  # noqa: E402
from cocotb.triggers import ClockCycles, Combine, FallingEdge  # noqa: E402


async def setup(dut, wiring="uart"):
    cocotb.start_soon(Clock(dut.clk, 20834, unit="ps").start())
    e1 = Chip(dut, "e1_", clock=False, pads=False)
    e2 = Chip(dut, "e2_", clock=False, pads=False)
    await Combine(cocotb.start_soon(e1.reset()), cocotb.start_soon(e2.reset()))
    cocotb.start_soon((pads if wiring == "uart" else pads_spi)(e1, e2))
    return e1, e2


async def pads(e1, e2):
    while True:
        await FallingEdge(e1.dut.clk)
        l1, l2 = e1.line(), e2.line()
        in1 = (l1 & ~0x02) | ((l2 & 1) << 1)
        in2 = (l2 & ~0x02) | ((l1 & 1) << 1)
        e1.samples.append(in1)
        e2.samples.append(in2)
        e1.sig("uio_in").value = in1
        e2.sig("uio_in").value = in2


async def pads_spi(e1, e2):
    SCK, MOSI, MISO, CS = 2, 3, 4, 5
    m2 = (1 << SCK) | (1 << MOSI) | (1 << CS)
    m1 = 1 << MISO
    while True:
        await FallingEdge(e1.dut.clk)
        l1, l2 = e1.line(), e2.line()
        in1 = (l1 & ~m1) | (l2 & m1)
        in2 = (l2 & ~m2) | (l1 & m2)
        e1.samples.append(in1)
        e2.samples.append(in2)
        e1.sig("uio_in").value = in1
        e2.sig("uio_in").value = in2


async def pads_usb(e1, e2, jp, kp):
    """One shared bus with pull-ups: J when nobody drives it."""
    m = (1 << jp) | (1 << kp)
    while True:
        await FallingEdge(e1.dut.clk)
        o1, oe1 = int(e1.sig("uio_out").value), int(e1.sig("uio_oe").value)
        o2, oe2 = int(e2.sig("uio_out").value), int(e2.sig("uio_oe").value)
        assert not (oe1 & oe2 & m), "bus contention"
        bus = (o1 & oe1 & m) | (o2 & oe2 & m) | ((1 << jp) & ~(oe1 | oe2))
        line = 0xFF & ~m | bus
        e1.samples.append(line)
        e2.samples.append(line)
        e1.sig("uio_in").value = line
        e2.sig("uio_in").value = line


def vote(B):
    N, K = (9, 1) if B >= 100 else (5, 1)
    return dict(N=N, K=K, S=B // (N + 1))


@cocotb.test()
async def test_e1_to_e2(dut):
    e1, e2 = await setup(dut)
    for B in (112, 24):
        await Combine(cocotb.start_soon(e1.start_uart(B=B, **vote(B))),
                      cocotb.start_soon(e2.start_uart(B=B, **vote(B))))
        t0 = len(e1.samples)
        data = [random.getrandbits(8) for _ in range(10)]
        send = cocotb.start_soon(e1.transfer(data, n_rx=0))
        got = await e2.read_rx(len(data), limit=200000)
        await send
        assert got == data, B
        await ClockCycles(dut.clk, 2 * B)
        assert uart.decode(e1.trace(0, t0), B) == data
        assert await e1.bus.read(R.STATUS) & R.ST_RXF_EMPTY, "e2 sent nothing"


@cocotb.test()
async def test_both_directions(dut):
    e1, e2 = await setup(dut)
    await Combine(cocotb.start_soon(e1.start_uart(B=112, **vote(112))),
                  cocotb.start_soon(e2.start_uart(B=112, **vote(112))))
    a = [random.getrandbits(8) for _ in range(12)]
    b = [random.getrandbits(8) for _ in range(12)]
    t1 = cocotb.start_soon(e1.transfer(a, n_rx=len(b)))
    t2 = cocotb.start_soon(e2.transfer(b, n_rx=len(a)))
    await Combine(t1, t2)
    assert t1.result() == b
    assert t2.result() == a


@cocotb.test()
async def test_baud_mismatch(dut):
    e1, e2 = await setup(dut)
    await Combine(cocotb.start_soon(e1.start_uart(B=111, **vote(111))),
                  cocotb.start_soon(e2.start_uart(B=113, **vote(113))))
    a = [random.getrandbits(8) for _ in range(8)]
    b = [random.getrandbits(8) for _ in range(8)]
    t1 = cocotb.start_soon(e1.transfer(a, n_rx=len(b)))
    t2 = cocotb.start_soon(e2.transfer(b, n_rx=len(a)))
    await Combine(t1, t2)
    assert t1.result() == b
    assert t2.result() == a


@cocotb.test()
async def test_spi_all_modes(dut):
    e1, e2 = await setup(dut, "spi")
    for mode in range(4):
        for msb in (1, 0):
            cpol, cpha = mode >> 1, mode & 1
            await e1.bus.write(R.RUN, 0)
            await e2.bus.write(R.RUN, 0)
            await e2.bus.load_program(pe_fw.spi_slave(cpol=cpol, cpha=cpha, msb=msb).words())
            await e2.bus.owners(R.owner_map(p4=R.OWN_TX))
            await e1.bus.load_program(pe_fw.spi_master(B=16, cpol=cpol, cpha=cpha, msb=msb).words())
            await e1.bus.owners(R.owner_map(p2=R.OWN_TX, p3=R.OWN_TX))
            await e1.bus.start()            # master first: CS and SCK idle before the slave arms
            await ClockCycles(dut.clk, 40)
            n = 6
            down = [random.getrandbits(8) for _ in range(n)]
            up = [random.getrandbits(8) for _ in range(n)]
            await e2.bus.set_addr(R.TXF)
            for d in up + [0xEE]:
                await e2.bus.write_same(d)
            await e2.bus.start()
            await ClockCycles(dut.clk, 40)
            await e1.bus.dmem_write(0, [n] + down)
            await e1.bus.write(R.DOORBELL, 0)
            for _ in range(3000):
                if await e1.bus.read(R.STATUS2) & R.S2_IRQ:
                    break
            else:
                raise AssertionError("no controller IRQ")
            await e1.bus.write(R.STATUS2, R.S2_IRQ)
            assert await e1.bus.dmem_read(0x80, n) == up, f"mode {mode} msb {msb}: master read"
            assert await e2.read_rx(n) == down, f"mode {mode} msb {msb}: slave read"
            await e2.bus.write(R.RUN, 0)


async def usb_wait(e, what, limit=8000):
    for _ in range(limit):
        if await e.bus.read(R.STATUS2) & R.S2_IRQ:
            await e.bus.write(R.STATUS2, R.S2_IRQ)
            return
    raise AssertionError(f"no IRQ: {what}")


async def usb_txn(e1, e2, tok, dpid=0, data=()):
    await e1.bus.write(R.RUN, 1)
    await e1.bus.dmem_write(0, [0] + list(tok) + [dpid, len(data)])
    if data:
        await e1.bus.dmem_write(0x10, list(data))
    await e1.bus.write(R.DOORBELL, 0)
    await usb_wait(e1, "host")


@cocotb.test()
async def test_usb_both_speeds(dut):
    F = pe_fw
    for name, B, jp, kp in (("FS", 4, 0, 1), ("LS", 32, 1, 0)):
        cocotb.start_soon(Clock(dut.clk, 20834, unit="ps").start())
        e1 = Chip(dut, "e1_", clock=False, pads=False)
        e2 = Chip(dut, "e2_", clock=False, pads=False)
        await Combine(cocotb.start_soon(e1.reset()), cocotb.start_soon(e2.reset()))
        t = cocotb.start_soon(pads_usb(e1, e2, jp, kp))
        own = {f"p{jp}": R.OWN_TX, f"p{kp}": R.OWN_TX}
        await e1.bus.load_program(F.usb_host(B=B, jp=jp, kp=kp).words())
        await e1.bus.owners(R.owner_map(**own))
        await e2.bus.load_program(F.usb_device(B=B, jp=jp, kp=kp).words())
        await e2.bus.owners(R.owner_map(**own))
        back = [random.getrandbits(8) for _ in range(8)]
        await e2.bus.dmem_write(F.UD_ADDR, [5, 0, 0, 0, len(back) + 1, F.PID_DATA1, F.PID_ACK, F.PID_NAK, 0])
        await e2.bus.dmem_write(F.UD_INDATA, back)
        await e2.bus.start()
        await e1.bus.start()
        await ClockCycles(dut.clk, 200)
        # SETUP and OUT: data arrives at the device, the host sees the ACK
        for pid, ep in ((F.PID_SETUP, 0), (F.PID_OUT, 2)):
            data = [random.getrandbits(8) for _ in range(8)]
            await usb_txn(e1, e2, usb.token(pid, 5, ep), F.PID_DATA0, data)
            await usb_wait(e2, f"{name} device")
            r = await e1.bus.dmem_read(F.UH_RPID, 2)
            assert r == [F.PID_ACK, F.UH_OK], f"{name}: host result {r}"
            m = await e2.bus.dmem_read(0, 9)
            assert m[F.UD_EVENT] == F.EV_OUT and m[F.UD_TOKEN] == pid and m[F.UD_EP] == ep, m
            assert await e2.bus.dmem_read(F.UD_RXDATA + 1, 8) == data, name
        # IN: data arrives at the host, the device sees the ACK
        await usb_txn(e1, e2, usb.token(F.PID_IN, 5, 2))
        await usb_wait(e2, f"{name} device IN")
        r = await e1.bus.dmem_read(F.UH_RPID, 3)
        assert r == [F.PID_DATA1, F.UH_OK, 11], f"{name}: {r}"
        assert await e1.bus.dmem_read(0x81, 8) == back, name
        m = await e2.bus.dmem_read(0, 9)
        assert m[F.UD_EVENT] == F.EV_IN_ACK and m[F.UD_INLEN] == 0, m
        # IN again: nothing ready -> NAK
        await ClockCycles(dut.clk, 300)
        await usb_txn(e1, e2, usb.token(F.PID_IN, 5, 2))
        assert await e1.bus.dmem_read(F.UH_RPID, 2) == [F.PID_NAK, F.UH_OK], name
        # another address: no answer, the host times out
        await usb_txn(e1, e2, usb.token(F.PID_IN, 6, 2))
        assert (await e1.bus.dmem_read(F.UH_STATUS, 1))[0] == F.UH_TIMEOUT, name
        t.cancel()
