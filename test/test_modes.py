# SPDX-License-Identifier: Apache-2.0
"""
test_modes.py: every UART frame format and SPI word size (run through test/tb.v).

What this file does
-------------------
UART uses pe_fw.uart_ex in loopback, P0 -> P1. Each frame is checked on the wire
(uart.decode_ex: edges on the bit grid, stop time) and at the host. SPI uses the
models in spi.py.

Tests
-----
1. test_uart_all_frames     5-9 data bits x none/even/odd/mark/space x stop 1, 1.5, 2
                            x normal/inverted line: 150 formats.
2. test_uart_msb            MSB-first, with and without parity.
3. test_uart_bad_parity     error on purpose: a wrong parity bit drops the frame
                            and counts it in dmem[1]; good frames still arrive.
4. test_uart_bad_stop_parity  a bad stop bit drops a frame in a parity format.
5. test_uart_fast           8E2 and 9O1 at B = 14 (3.4 Mbaud, 48 MHz), full duplex.
6. test_uart_8bit_compat    the original pe_fw.uart at 5, 6, 7 bits through NBITS.
7. test_spi_master_words    chip as master, 4/5/7/9/12/16-bit words, all modes and orders.
8. test_spi_slave_words     chip as slave, 3/4/5/7-bit words, all modes and orders.
9. test_tx_inv_spi          TX INV inverts data but not the SPI clock.
"""

import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "common"))

import cocotb  # noqa: E402
import pe_build  # noqa: E402
import pe_fw  # noqa: E402
import pe_host as R  # noqa: E402
import pe_isa as I  # noqa: E402
import spi  # noqa: E402
import uart  # noqa: E402
from chip import Chip  # noqa: E402
from cocotb.triggers import ClockCycles  # noqa: E402

SCK, MOSI, MISO, CS = 2, 3, 4, 5
PN = {0: "N", 1: "E", 2: "O", 3: "M", 4: "S"}


async def uart_chip(dut, loop=True, **kw):
    chip = Chip(dut)
    await chip.reset()
    if loop:
        chip.loop.append((0, 1))
    if kw.get("inv"):
        chip.ext = 0x00                     # idle low before the pin is driven
        chip.drive[1] = 0
    await chip.bus.write(R.RUN, 0)
    await chip.bus.load_program(pe_fw.uart_ex(**kw).words())
    await chip.bus.owners(R.owner_map(p0=R.OWN_TX))
    if kw.get("inv"):
        del chip.drive[1]
    await chip.bus.start()
    await ClockCycles(dut.clk, 40)
    return chip


def to_bytes(words, bits):
    if bits <= 8:
        return list(words)
    out = []
    for w in words:
        out += [w & 0xFF, w >> 8]
    return out


def expected_word(d, bits, parity, msb=0):
    if parity == pe_fw.PAR_NONE:
        return d
    p = uart.parity_of(d, parity)
    return (d << 1) | p if msb else d | (p << bits)


async def check_format(dut, B, bits, parity, stop_b2, inv, msb=0, n=2):
    tag = f"{bits}{PN[parity]}{stop_b2 / 2} inv={inv} msb={msb} B={B}"
    S = B // 10
    chip = await uart_chip(dut, B=B, N=9, K=1, S=S, bits=bits, parity=parity, stop_b2=stop_b2, inv=inv, msb=msb)
    words = [random.getrandbits(bits) for _ in range(n)]
    data = to_bytes(words, bits)
    t0 = len(chip.samples)
    got = await chip.transfer(data, n_rx=len(data))
    assert got == data, f"{tag}: host got {got}, sent {data}"
    await ClockCycles(dut.clk, 3 * B)
    wb = bits + (parity != pe_fw.PAR_NONE)
    wire = uart.decode_ex(chip.trace(0, t0), B, wb, stop_b2, inv, msb)
    exp = [expected_word(w, bits, parity, msb) for w in words]
    assert wire[:n] == exp, f"{tag}: wire {wire}, expected {exp}"
    await chip.bus.write(R.RUN, 0)


@cocotb.test()
async def test_uart_all_frames(dut):
    for bits in (5, 6, 7, 8, 9):
        for parity in range(5):
            for stop_b2 in (2, 3, 4):
                for inv in (0, 1):
                    await check_format(dut, 32, bits, parity, stop_b2, inv)


@cocotb.test()
async def test_uart_msb(dut):
    for bits, parity in ((8, 0), (8, 1), (7, 2), (5, 3), (9, 4)):
        await check_format(dut, 32, bits, parity, 2, 0, msb=1)


async def drive_frames(chip, B, frames):
    for f in frames:
        await uart.drive_bits(chip, 1, f, B)


@cocotb.test()
async def test_uart_bad_parity(dut):
    B = 32
    for parity in (1, 2, 3, 4):
        chip = await uart_chip(dut, loop=False, B=B, N=9, K=1, S=3, bits=8, parity=parity)
        good1, bad, good2 = 0x35, 0x5A, 0xC4
        word = lambda d, wrong=0: d | ((uart.parity_of(d, parity) ^ wrong) << 8)  # noqa: E731
        chip.drive[1] = 1
        await ClockCycles(dut.clk, 3 * B)
        await drive_frames(chip, B, [uart.frame_ex(word(good1), 9), uart.frame_ex(word(bad, 1), 9),
                                     uart.frame_ex(word(good2), 9)])
        await ClockCycles(dut.clk, 3 * B)
        assert await chip.read_rx(2) == [good1, good2], f"parity {parity}"
        assert await chip.bus.read(R.STATUS) & R.ST_RXF_EMPTY, "bad frame was kept"
        await chip.bus.write(R.RUN, 0)
        assert (await chip.bus.dmem_read(1, 1))[0] == 1, "parity error not counted"


@cocotb.test()
async def test_uart_bad_stop_parity(dut):
    B = 32
    chip = await uart_chip(dut, loop=False, B=B, N=9, K=1, S=3, bits=7, parity=1)
    w = lambda d: d | (uart.parity_of(d, 1) << 7)  # noqa: E731
    chip.drive[1] = 1
    await ClockCycles(dut.clk, 3 * B)
    await drive_frames(chip, B, [uart.frame_ex(w(0x11), 8)])
    bad = uart.frame_ex(w(0x22), 8)
    bad[-1] = 0
    await drive_frames(chip, B, [bad, [1, 1], uart.frame_ex(w(0x33), 8)])
    await ClockCycles(dut.clk, 3 * B)
    assert await chip.read_rx(2) == [0x11, 0x33]


@cocotb.test()
async def test_uart_fast(dut):
    for bits, parity, stop_b2 in ((8, 1, 4), (9, 2, 2)):
        await check_format(dut, 14, bits, parity, stop_b2, 0, n=6)


@cocotb.test()
async def test_uart_8bit_compat(dut):
    """The hardware-timed firmware also works at other lengths through NBITS."""
    for bits in (5, 6, 7):
        B = 32
        a = pe_fw.uart(B=B, N=9, K=1, S=3)
        words = a.words()
        # patch the two FRAME writes to the new length
        for i, w in enumerate(words):
            if w >> 26 == I.OP["ucfg"] and (w >> 16) & 7 == 2:
                hold = w & 2
                words[i] = (w & ~0xFFFF) | hold | (bits << 4)
        chip = Chip(dut)
        await chip.reset()
        chip.loop.append((0, 1))
        await chip.bus.load_program(words)
        await chip.bus.owners(R.owner_map(p0=R.OWN_TX))
        await chip.bus.start()
        await ClockCycles(dut.clk, 40)
        data = [random.getrandbits(bits) for _ in range(4)]
        t0 = len(chip.samples)
        assert await chip.transfer(data) == data, bits
        await ClockCycles(dut.clk, 3 * B)
        assert uart.decode_ex(chip.trace(0, t0), B, bits)[:4] == data, bits
        await chip.bus.write(R.RUN, 0)


async def master_words(chip, words):
    data = [len(words)]
    for w in words:
        data += [w & 0xFF, w >> 8]
    await chip.bus.dmem_write(0, data)
    await chip.bus.write(R.DOORBELL, 0)
    for _ in range(4000):
        if await chip.bus.read(R.STATUS2) & R.S2_IRQ:
            break
    else:
        raise AssertionError("no controller IRQ")
    await chip.bus.write(R.STATUS2, R.S2_IRQ)
    raw = await chip.bus.dmem_read(0x80, 2 * len(words))
    return [raw[2 * i] | raw[2 * i + 1] << 8 for i in range(len(words))]


@pe_build.test("spi")
async def test_spi_master_words(dut):
    for bits in (4, 5, 7, 9, 12, 16):
        for mode in range(4):
            for msb in (1, 0):
                cpol, cpha = mode >> 1, mode & 1
                chip = Chip(dut)
                await chip.reset()
                await chip.bus.load_program(
                    pe_fw.spi_master_words(B=4, sdly=2, cpol=cpol, cpha=cpha, msb=msb, bits=bits).words())
                await chip.bus.owners(R.owner_map(p2=R.OWN_TX, p3=R.OWN_TX))
                await chip.bus.start()
                await ClockCycles(dut.clk, 30)
                out = [random.getrandbits(bits) for _ in range(3)]
                rep = [random.getrandbits(bits) for _ in range(3)]
                dev = spi.SpiSlave(SCK, MOSI, MISO, CS, cpol, cpha, msb, tx=rep, bits=bits)
                chip.devices.append(dev)
                got = await master_words(chip, out)
                tag = f"bits {bits} mode {mode} msb {msb}"
                assert dev.rx == out, f"{tag}: device got {dev.rx}, sent {out}"
                assert got == rep, f"{tag}: chip got {got}, device sent {rep}"
                assert len(dev.edges) == 2 * bits * 3, tag
                await chip.bus.write(R.RUN, 0)


@pe_build.test("spi")
async def test_spi_slave_words(dut):
    for bits in (3, 4, 5, 7):
        for mode in range(4):
            for msb in (1, 0):
                cpol, cpha = mode >> 1, mode & 1
                resp = [random.getrandbits(bits) for _ in range(3)]
                chip = Chip(dut)
                await chip.reset()
                chip.drive.update({SCK: cpol, MOSI: 0, CS: 1})
                await chip.bus.load_program(pe_fw.spi_slave(cpol=cpol, cpha=cpha, msb=msb, bits=bits).words())
                await chip.bus.owners(R.owner_map(p4=R.OWN_TX))
                await chip.bus.set_addr(R.TXF)
                for d in resp + [0]:
                    await chip.bus.write_same(d)
                await chip.bus.start()
                await ClockCycles(dut.clk, 30)
                data = [random.getrandbits(bits) for _ in range(3)]
                got = await spi.master_xfer(chip, data, h=8, cpol=cpol, cpha=cpha, msb=msb, bits=bits)
                tag = f"bits {bits} mode {mode} msb {msb}"
                assert got == resp, f"{tag}: MISO {got}, expected {resp}"
                assert await chip.read_rx(3) == data, tag
                await chip.bus.write(R.RUN, 0)


@pe_build.test("spi")
async def test_tx_inv_spi(dut):
    chip = Chip(dut)
    await chip.reset()
    a = pe_fw.spi_master(B=4, sdly=2)
    words = a.words()
    for i, w in enumerate(words):
        if w >> 26 == I.OP["ucfg"] and (w >> 20) & 7 == I.TX and (w >> 16) & 7 == I.TX_FRAME:
            words[i] = w | 4
    await chip.bus.load_program(words)
    await chip.bus.owners(R.owner_map(p2=R.OWN_TX, p3=R.OWN_TX))
    await chip.bus.start()
    await ClockCycles(dut.clk, 30)
    dev = spi.SpiSlave(SCK, MOSI, MISO, CS, tx=[0xA5])
    chip.devices.append(dev)
    await chip.bus.dmem_write(0, [1, 0x3C])
    await chip.bus.write(R.DOORBELL, 0)
    for _ in range(2000):
        if await chip.bus.read(R.STATUS2) & R.S2_IRQ:
            break
    assert dev.rx == [0x3C ^ 0xFF], f"device got {dev.rx}"
    assert len(dev.edges) == 16
