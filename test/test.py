# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0
"""
test.py: top-level tests for tt_um_sahil_protoemu (run through test/tb.v).

What this file does
-------------------
Tests the whole chip only through its pins, the way the RP2040 host will
use it: the 4-bit host bus on ui_in/uo_out (pe_host.HostBus) loads
firmware, configures contexts/pins/vote and moves bytes through the FIFOs.
The protocol pins are uio. The same tests run for RTL and for gate level
(make GATES=yes).

Tests
-----
1. test_registers          reset state (no pin drives), ID/VERSION, RW
                           readback, unmapped address reads 0.
2. test_uart_loopback      UART TX on ctx0 (P0) and vote-based RX on ctx1
                           (P1), P0 wired to P1 in the test. 16 random bytes
                           host -> TXF -> P0 -> P1 -> RXF -> host. The TX
                           waveform is checked cycle-exact (B = 112).
3. test_uart_115200        the same at 115200 baud (B = 434 cycles at 50 MHz,
                           RX bit period 9*48+4 = 436, 0.46% mismatch).
4. test_uart_baud_error    the TX bit period is 2% off the RX bit period
                           (+/-2 cycles at B = 112); bytes still arrive.
5. test_framing_error      the test drives P1 directly: good byte, byte with
                           stop bit = 0, good byte. Only the two good bytes
                           reach the RX FIFO.
6. test_imem_err_irq       an IMEM commit while a context runs is rejected:
                           STATUS.imem_err and the IRQ pin go high; ERR_CLR
                           clears them.
7. test_open_drain         P2 in open-drain mode: uio_out[2] stays 0 and
                           uio_oe[2] toggles with the firmware.
8. test_owner_map          ctx1 writing a ctx0-owned pin is ignored and
                           sets STATUS.owner_err.
"""

import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "common"))

import cocotb  # noqa: E402
import pe_fw  # noqa: E402
import pe_host as R  # noqa: E402
import pe_isa as I  # noqa: E402
import uart  # noqa: E402
from cocotb.clock import Clock  # noqa: E402
from cocotb.triggers import ClockCycles, FallingEdge  # noqa: E402


class Chip:
    """Reset, host bus, uio loopback/drive and uio sampling."""

    def __init__(self, dut):
        self.dut = dut
        self.bus = R.HostBus(dut)
        self.loop = []          # (src_pin, dst_pin) wired outside the chip
        self.ext = 0xFF         # level on undriven uio inputs (pull-ups)
        self.samples = []

    async def reset(self):
        dut = self.dut
        cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
        dut.ena.value = 1
        dut.ui_in.value = 0
        dut.uio_in.value = 0xFF
        dut.rst_n.value = 0
        await ClockCycles(dut.clk, 10)
        dut.rst_n.value = 1
        await ClockCycles(dut.clk, 5)
        cocotb.start_soon(self._pads())

    async def _pads(self):
        dut = self.dut
        while True:
            await FallingEdge(dut.clk)
            out, oe = int(dut.uio_out.value), int(dut.uio_oe.value)
            line = (out & oe) | (self.ext & ~oe)   # driven pins win, others float to ext
            self.samples.append(line & 0xFF)
            x = line
            for s, t in self.loop:
                x = (x & ~(1 << t)) | (((line >> s) & 1) << t)
            dut.uio_in.value = x & 0xFF

    def trace(self, pin, start=0):
        return [(s >> pin) & 1 for s in self.samples[start:]]

    async def start_uart(self, B_tx, N, S, K=1):
        bus = self.bus
        await bus.write(R.RUN, 0)
        await bus.load_program(pe_fw.uart_tx(0, B_tx, base=0), start=0)
        await bus.load_program(pe_fw.uart_rx(1, N, S, base=20), start=20)
        await bus.write(R.PIN_OWNER, 0b10)              # P1 -> ctx1, others ctx0
        await bus.setup_ctx(0, start_pc=0)
        await bus.setup_ctx(1, start_pc=20, vote=(N, K, S))
        await bus.write(R.RESTART, 0b11)
        await bus.write(R.RUN, 0b11)

    async def transfer(self, data):
        """Send bytes through TXF while collecting RXF, using only the host bus."""
        bus = self.bus
        got, todo = [], list(data)
        for _ in range(100000):
            st = await bus.read(R.STATUS)
            if not st & R.ST_RXF_EMPTY:
                got.append(await bus.read(R.RXF))
                if len(got) == len(data):
                    return got
            elif todo and not st & R.ST_TXF_FULL:
                await bus.write(R.TXF, todo.pop(0))
        raise AssertionError(f"timeout, got {got}")


async def uart_run(dut, B_tx, N, S, nbytes, check_tx=True):
    chip = Chip(dut)
    await chip.reset()
    chip.loop = [(0, 1)]
    await chip.start_uart(B_tx, N, S)
    t0 = len(chip.samples)
    data = [random.getrandbits(8) for _ in range(nbytes)]
    got = await chip.transfer(data)
    assert got == data, f"sent {data}\ngot  {got}"
    assert await chip.bus.read(R.STATUS) & 0xF0 == 0, "no error flags expected"
    if check_tx:
        await ClockCycles(dut.clk, 2 * B_tx)
        assert uart.decode(chip.trace(0, t0), B_tx) == data
    return chip


@cocotb.test()
async def test_registers(dut):
    chip = Chip(dut)
    await chip.reset()
    bus = chip.bus
    assert int(dut.uio_oe.value) == 0, "no protocol pin may drive after reset"
    assert (int(dut.uo_out.value) >> 6) == 0
    assert await bus.read(R.ID) == 0x50
    assert await bus.read(R.VERSION) == 0x01
    for a, x in ((R.PIN_OD, 0xA5), (R.IRQ_EN, 0x05), (R.ctx_reg(1, R.VOTE_S_LO), 0x3C),
                 (R.ctx_reg(0, R.START_PC), 0x2A)):
        await bus.write(a, x)
        assert await bus.read(a) == x
    await bus.write(R.PIN_OD, 0)
    await bus.write(R.IRQ_EN, 0)
    assert await bus.read(0x7F) == 0


@cocotb.test()
async def test_uart_loopback(dut):
    await uart_run(dut, B_tx=112, N=9, S=12, nbytes=16)


@cocotb.test()
async def test_uart_115200(dut):
    await uart_run(dut, B_tx=434, N=9, S=48, nbytes=4)


@cocotb.test()
async def test_uart_baud_error(dut):
    for B_tx in (110, 114):  # RX expects 112: -1.8% / +1.8%
        await uart_run(dut, B_tx=B_tx, N=9, S=12, nbytes=6)


@cocotb.test()
async def test_framing_error(dut):
    chip = Chip(dut)
    await chip.reset()
    B = pe_fw.rx_bit_period(9, 12)
    await chip.start_uart(B, 9, 12)
    await chip.bus.write(R.RUN, 0b10)       # RX only; the test drives P1
    good1, bad, good2 = 0x5A, 0xC3, 0x81
    bits = [1] * 3 + uart.frame_bits(good1) + [1] * 2 + uart.frame_bits(bad, stop=0) + [0] * 2
    bits += [1] * 4 + uart.frame_bits(good2) + [1] * 3
    for b in bits:
        chip.ext = 0xFD | (b << 1)
        await ClockCycles(dut.clk, B)
    st = await chip.bus.read(R.STATUS)
    assert not st & R.ST_RXF_EMPTY
    got = [await chip.bus.read(R.RXF)]
    got.append(await chip.bus.read(R.RXF))
    assert got == [good1, good2], got
    assert await chip.bus.read(R.STATUS) & R.ST_RXF_EMPTY, "framing-error byte must be dropped"


@cocotb.test()
async def test_imem_err_irq(dut):
    chip = Chip(dut)
    await chip.reset()
    bus = chip.bus
    await bus.load_program([I.jmp(0)])
    await bus.write(R.RESTART, 1)
    await bus.write(R.RUN, 1)
    await bus.write(R.IRQ_EN, 0x4)
    assert (int(dut.uo_out.value) >> 5) & 1 == 0
    await bus.load_program([I.nop()], start=0)           # rejected: ctx0 is running
    assert await bus.read(R.STATUS) & R.ST_IMEM_ERR
    assert (int(dut.uo_out.value) >> 5) & 1 == 1, "IRQ pin"
    await bus.write(R.ERR_CLR, 0x80)
    assert not await bus.read(R.STATUS) & R.ST_IMEM_ERR
    assert (int(dut.uo_out.value) >> 5) & 1 == 0
    await bus.write(R.RUN, 0)


@cocotb.test()
async def test_open_drain(dut):
    chip = Chip(dut)
    await chip.reset()
    bus = chip.bus
    await bus.load_program([I.set_(2, I.SRC_0), I.delay(9), I.set_(2, I.SRC_1), I.delay(9), I.jmp(0)])
    await bus.write(R.PIN_OD, 0x04)
    await bus.write(R.RESTART, 1)
    t0 = len(chip.samples)
    await bus.write(R.RUN, 1)
    oes = []
    for _ in range(200):
        await FallingEdge(dut.clk)
        assert (int(dut.uio_out.value) >> 2) & 1 == 0, "open-drain pin must never drive high"
        oes.append((int(dut.uio_oe.value) >> 2) & 1)
    assert 0 in oes and 1 in oes, "oe must toggle"
    assert set(chip.trace(2, t0)) == {0, 1}
    await bus.write(R.RUN, 0)


@cocotb.test()
async def test_owner_map(dut):
    chip = Chip(dut)
    await chip.reset()
    bus = chip.bus
    await bus.load_program([I.setoe(3, 1), I.set_(3, I.SRC_0), I.jmp(2)])
    await bus.write(R.PIN_OWNER, 0x00)                 # every pin belongs to ctx0
    await bus.setup_ctx(1, start_pc=0)
    await bus.write(R.RESTART, 0b10)
    await bus.write(R.RUN, 0b10)                       # run the program on ctx1
    await ClockCycles(dut.clk, 50)
    assert (int(dut.uio_oe.value) >> 3) & 1 == 0, "ctx1 must not control P3"
    assert await bus.read(R.STATUS) & R.ST_OWNER_ERR
    await bus.write(R.RUN, 0)
    await bus.write(R.ERR_CLR, 0x40)
    assert not await bus.read(R.STATUS) & R.ST_OWNER_ERR
