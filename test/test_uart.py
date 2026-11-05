# SPDX-License-Identifier: Apache-2.0
"""
test_uart.py: top-level UART tests (v2), run through test/tb.v.

What this file does
-------------------
The scenarios of the v1 UART tests are kept (loopback, 115200, baud error,
framing error), plus speed, error and corner cases. Firmware is
pe_fw.uart (docs/spec/isa.md Section 5): TX on P0, RX on P1, both on one
chip, full duplex. TX waveforms are checked cycle-exact by uart.decode (every
edge on the bit grid, start and stop exact). RX input is either P0 wired to
P1 (loopback) or driven by the test (uart.drive_bits).

Tests
-----
1.  test_uart_loopback     B = 112, 16 random bytes, TX cycle-exact.
2.  test_uart_115200       B = 417 (115200 baud at 48 MHz), vote (9, 46).
3.  test_uart_speeds       B = 50, 24, 16, 12 (0.96, 2, 3, 4 Mbaud) loopback.
4.  test_uart_baud_error   TX 110 / 114 vs RX 112 (+-1.8%).
5.  test_uart_rx_baud_tol  RX at B = 112 fed at B = 109 / 115 (+-2.7%).
6.  test_framing_error     good, bad stop bit, good: the bad byte is dropped.
7.  test_break             line low for 30 bits (break) then a good byte:
                           nothing but the good byte arrives.
8.  test_glitch_reject     low pulses shorter than DUR on idle RX: no byte.
9.  test_noise_in_bits     a short inverted pulse inside every bit: the vote
                           still gives the right bytes.
10. test_back_to_back      RX frames with exactly 1 stop bit, no idle, at
                           B = 112 and B = 12 (AUTOARM re-arm in time).
11. test_full_duplex       TX and RX at the same time with different data
                           (no loopback): TX decoded cycle-exact, RX correct.
12. test_rx_overrun        the host stops reading: RX FIFO fills, firmware
                           blocks in fput, the RX unit overruns ->
                           STATUS2.rx_overrun; the first 8 bytes are intact;
                           traffic resumes after the host reads.
13. test_tx_underrun       ETX starts the TX unit with an empty TX FIFO ->
                           start dropped, STATUS2.tx_underrun.
14. test_tx_idle_gaps      the host sends with pauses: every frame still
                           starts with an exact start bit; line idles high.
15. test_uart_restart      RESTART in the middle of a frame; after restart
                           the next transfer is clean.
"""

import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "common"))

import cocotb  # noqa: E402
import pe_host as R  # noqa: E402
import pe_isa as I  # noqa: E402
import uart  # noqa: E402
from chip import Chip  # noqa: E402
from cocotb.triggers import ClockCycles  # noqa: E402
from pe_isa import Asm  # noqa: E402


def rx_vote_for(B):
    """A vote that fits in B cycles: (N, K, S)."""
    N, K = (9, 1) if B >= 100 else (5, 1) if B >= 24 else (3, 0)
    return N, K, B // (N + 1)          # samples centred in the bit


async def loopback(dut, B, nbytes, B_rx=None, check_tx=True):
    chip = Chip(dut)
    await chip.reset()
    chip.loop = [(0, 1)]
    N, K, S = rx_vote_for(B if B_rx is None else B_rx)
    await chip.start_uart(B=B, N=N, K=K, S=S, B_rx=B_rx)
    t0 = len(chip.samples)
    data = [random.getrandbits(8) for _ in range(nbytes)]
    got = await chip.transfer(data)
    assert got == data, f"B={B}\nsent {data}\ngot  {got}"
    assert await chip.bus.read(R.STATUS) & 0xF0 == 0, "no error flags expected"
    assert await chip.bus.read(R.STATUS2) == 0
    if check_tx:
        await ClockCycles(dut.clk, 2 * B)
        assert uart.decode(chip.trace(0, t0), B) == data
    return chip


async def rx_only(dut, B=112, **kw):
    chip = Chip(dut)
    await chip.reset()
    chip.drive[1] = 1
    N, K, S = rx_vote_for(B)
    await chip.start_uart(B=B, N=N, K=K, S=S, tx=False, **kw)
    await ClockCycles(dut.clk, 20)
    return chip


@cocotb.test()
async def test_uart_loopback(dut):
    await loopback(dut, B=112, nbytes=16)


@cocotb.test()
async def test_uart_115200(dut):
    chip = Chip(dut)
    await chip.reset()
    chip.loop = [(0, 1)]
    await chip.start_uart(B=417, N=9, K=1, S=46)
    t0 = len(chip.samples)
    data = [0x00, 0xFF, 0x55, 0xA5]
    assert await chip.transfer(data) == data
    await ClockCycles(dut.clk, 2 * 417)
    assert uart.decode(chip.trace(0, t0), 417) == data


@cocotb.test()
async def test_uart_speeds(dut):
    for B in (50, 24, 16, 12):
        await loopback(dut, B=B, nbytes=8)


@cocotb.test()
async def test_uart_baud_error(dut):
    for B_tx in (110, 114):  # RX expects 112: -1.8% / +1.8%
        await loopback(dut, B=B_tx, B_rx=112, nbytes=6)


@cocotb.test()
async def test_uart_rx_baud_tol(dut):
    for B_line in (109, 115):  # -2.7% / +2.7% against B = 112
        chip = await rx_only(dut, 112)
        data = [random.getrandbits(8) for _ in range(6)]
        await uart.drive_bits(chip, 1, [1] * 2 + uart.stream_bits(data, idle=1) + [1] * 2, B_line)
        assert await chip.read_rx(6) == data, B_line


@cocotb.test()
async def test_framing_error(dut):
    chip = await rx_only(dut)
    good1, bad, good2 = 0x5A, 0xC3, 0x81
    bits = [1] * 3 + uart.frame_bits(good1) + [1] * 2 + uart.frame_bits(bad, stop=0) + [0] * 2
    bits += [1] * 4 + uart.frame_bits(good2) + [1] * 3
    await uart.drive_bits(chip, 1, bits, 112)
    assert await chip.read_rx(2) == [good1, good2]
    await ClockCycles(dut.clk, 500)
    assert await chip.bus.read(R.STATUS) & R.ST_RXF_EMPTY, "framing-error byte must be dropped"
    assert await chip.bus.read(R.STATUS2) == 0


@cocotb.test()
async def test_break(dut):
    chip = await rx_only(dut)
    await uart.drive_bits(chip, 1, [1] * 2 + [0] * 30 + [1] * 3 + uart.frame_bits(0x3C) + [1] * 2, 112)
    assert await chip.read_rx(1) == [0x3C]
    await ClockCycles(dut.clk, 300)
    assert await chip.bus.read(R.STATUS) & R.ST_RXF_EMPTY, "the break must not produce bytes"


@cocotb.test()
async def test_glitch_reject(dut):
    chip = await rx_only(dut)
    for w in (1, 5, 20, 55):                     # all shorter than DUR = 56
        chip.drive[1] = 0
        await ClockCycles(dut.clk, w)
        chip.drive[1] = 1
        await ClockCycles(dut.clk, 300)
    assert await chip.bus.read(R.STATUS) & R.ST_RXF_EMPTY, "glitches must not start a byte"
    await uart.drive_bits(chip, 1, uart.frame_bits(0x96) + [1] * 2, 112)
    assert await chip.read_rx(1) == [0x96]


@cocotb.test()
async def test_noise_in_bits(dut):
    chip = await rx_only(dut)
    data = [0x00, 0xFF, 0x5A, 0xA5, 0x0F]
    # 2 of the 9 vote samples (12 cycles apart) hit by a 20-cycle pulse in every bit
    # (start bit included, but after the detector has fired at DUR = 56)
    await uart.drive_bits(chip, 1, [1] * 2 + uart.stream_bits(data, idle=1), 112, glitch=(70, 20))
    assert await chip.read_rx(len(data)) == data


@cocotb.test()
async def test_back_to_back(dut):
    for B in (112, 12):
        chip = await rx_only(dut, B)
        data = [random.getrandbits(8) for _ in range(10)]
        await uart.drive_bits(chip, 1, [1] * 2 + uart.stream_bits(data, idle=0) + [1] * 3, B)
        assert await chip.read_rx(10) == data, B
        assert await chip.bus.read(R.STATUS2) == 0


@cocotb.test()
async def test_full_duplex(dut):
    chip = Chip(dut)
    await chip.reset()
    chip.drive[1] = 1
    await chip.start_uart(B=112, N=9, K=1, S=12)
    t0 = len(chip.samples)
    tx_data = [random.getrandbits(8) for _ in range(8)]
    rx_data = [random.getrandbits(8) for _ in range(8)]
    drv = cocotb.start_soon(uart.drive_bits(chip, 1, [1] * 3 + uart.stream_bits(rx_data, idle=0) + [1] * 2, 112))
    got = await chip.transfer(tx_data, n_rx=8)
    await drv
    assert got == rx_data
    await ClockCycles(dut.clk, 2 * 112)
    assert uart.decode(chip.trace(0, t0), 112) == tx_data


@cocotb.test()
async def test_rx_overrun(dut):
    chip = await rx_only(dut)
    data = [random.getrandbits(8) for _ in range(12)]
    await uart.drive_bits(chip, 1, [1] * 2 + uart.stream_bits(data, idle=1), 112)
    await ClockCycles(dut.clk, 500)
    assert await chip.bus.read(R.STATUS2) & R.S2_RX_OVR, "RX unit overrun expected"
    assert await chip.bus.read(R.STATUS) & R.ST_RXF_FULL
    got = await chip.read_rx(8)
    assert got == data[:8], "bytes before the overrun are intact"
    await chip.bus.write(R.STATUS2, R.S2_RX_OVR)
    assert not await chip.bus.read(R.STATUS2) & R.S2_RX_OVR
    # traffic resumes (the firmware's pending fput completes first)
    await ClockCycles(dut.clk, 200)
    while not await chip.bus.read(R.STATUS) & R.ST_RXF_EMPTY:
        await chip.bus.read(R.RXF)
    more = [0x11, 0x22]
    await uart.drive_bits(chip, 1, uart.stream_bits(more, idle=1), 112)
    assert await chip.read_rx(2) == more


@cocotb.test()
async def test_tx_underrun(dut):
    chip = Chip(dut)
    await chip.reset()
    bus = chip.bus
    a = Asm()
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(0)).ucfg(I.TX, I.TX_BIT, 20)
    a.ucfg(I.ETX_U, I.D_COND, I.cond(2, I.FALL, start=1))     # P2 falling edge starts TX
    a.evt(I.DET_TX, I.EV_ARM)
    a.evt(I.DET_TX, I.EV_WAITC)
    a.mov(0, 0x77).mov(1, 0).st(0, 1)
    a.L("e").jmp("e")
    await bus.owners(R.owner_map(p0=R.OWN_TX))
    await bus.load_program(a.words())
    await bus.start()
    await ClockCycles(dut.clk, 30)
    chip.drive[2] = 0                         # the event fires, the TX FIFO is empty
    await ClockCycles(dut.clk, 30)
    assert await bus.read(R.STATUS2) & R.S2_TX_UNDER
    cs = await bus.read(R.CSTAT)
    assert not cs & R.CS_ETX, "waitc cleared the flag"
    await bus.write(R.RUN, 0)
    assert await bus.dmem_read(0, 1) == [0x77], "the flag was still set"
    assert set(chip.trace(0)) == {1}, "the TX unit did not start"


@cocotb.test()
async def test_tx_idle_gaps(dut):
    chip = Chip(dut)
    await chip.reset()
    chip.loop = [(0, 1)]
    await chip.start_uart(B=50, N=5, K=1, S=9)
    t0 = len(chip.samples)
    data = []
    for i in range(4):
        d = [random.getrandbits(8) for _ in range(2)]
        data += d
        assert await chip.transfer(d) == d
        await ClockCycles(dut.clk, 37 * (i + 1))      # irregular pauses
    await ClockCycles(dut.clk, 100)
    tr = chip.trace(0, t0)
    assert uart.decode(tr, 50) == data
    assert tr[-50:] == [1] * 50, "line idles high"


@cocotb.test()
async def test_uart_restart(dut):
    chip = Chip(dut)
    await chip.reset()
    chip.loop = [(0, 1)]
    await chip.start_uart(B=112, N=9, K=1, S=12)
    await chip.bus.write(R.TXF, 0x00)
    await ClockCycles(dut.clk, 500)                    # mid-frame
    await chip.bus.write(R.RUN, 0)
    await chip.bus.write(R.RESTART, 1)
    while not await chip.bus.read(R.STATUS) & R.ST_RXF_EMPTY:
        await chip.bus.read(R.RXF)
    await chip.start_uart(B=112, N=9, K=1, S=12)
    t0 = len(chip.samples)
    data = [0x12, 0x34, 0x56]
    assert await chip.transfer(data) == data
    await ClockCycles(dut.clk, 2 * 112)
    assert uart.decode(chip.trace(0, t0), 112) == data
