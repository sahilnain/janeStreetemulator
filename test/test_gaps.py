# SPDX-License-Identifier: Apache-2.0
"""
test_gaps.py: protocol features added after the first full v2 (run through test/tb.v).

What this file does
-------------------
Each test drives the firmware with an independent Python peer (spi.py, i2c.py, usb.py) or by
wires from the testbench. test_cross.py runs the same firmware between the RTL and the model.

Tests
-----
UART
1. test_uart_odd_b           uart_ex with B = 35 and 417: the line stays on the bit grid.
2. test_uart_break_rx        a long low line counts one break, with an IRQ; a 0x00 frame with a good
                             stop bit does not, nor does a bad stop bit with data.
3. test_uart_break_tx        a doorbell sends a break of n bit times, and frames still work after it.
SPI
4. test_spi_stream           CHAIN master: 40 bytes at B = 4 in all modes, SCK with no pause.
5. test_spi_stream_fast      9 bytes at B = 2 (24 MHz), SCK with no pause.
6. test_spi_3wire_master     one data pin: write, then read, against a Spi3Slave; no contention.
7. test_spi_3wire_slave      one data pin: a testbench master writes and reads, all modes.
8. test_spi_slave_words      slave with 9, 12, 15, 16 bits, all modes, both directions.
I2C
9. test_i2c_10bit_master     10-bit write, then repeated-START read; the wrong address is NACKed.
10. test_i2c_hs_master       the master code at F/S speed, then the data bytes at hs_B = 14 (3 MHz).
11. test_i2c_arbitration     a rival master on the bus: the chip loses at a bit, stops driving at once
                             and reports 3 with no STOP; or it wins and finishes; it waits for a free bus.
12. test_i2c_10bit_slave     the chip answers a 10-bit address, write and read; others are ignored.
13. test_i2c_slave_stretch   SCL is held low after each ACK while the firmware is busy.
14. test_i2c_general_call    address 0 is acknowledged, data at I2C_GC_MEM, with an IRQ.
USB
15. test_usb_device_suspend  CSTAT.ETX shows idle J; a SOF clears it; a resume does not disturb the
                             device, and the next transaction works.
16. test_usb_host_resume     the host's K for UH_N * 256 cycles, then an EOP, then a transaction.
"""

import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "common"))

import cocotb  # noqa: E402
import i2c  # noqa: E402
import pe_fw  # noqa: E402
import pe_host as R  # noqa: E402
import spi  # noqa: E402
import uart  # noqa: E402
import usb  # noqa: E402
from chip import Chip  # noqa: E402
from cocotb.triggers import ClockCycles, FallingEdge  # noqa: E402
from test_usb import device as usb_device_chip, host as usb_host_chip, send_token_data, tok  # noqa: E402

SCK, MOSI, MISO, CS = 2, 3, 4, 5


async def fresh(dut, words, owners=0, od=0):
    chip = Chip(dut)
    await chip.reset()
    await chip.bus.write(R.RUN, 0)
    if od:
        await chip.bus.write(R.PIN_OD, od)
    await chip.bus.load_program(words)
    await chip.bus.owners(owners)
    return chip


async def wait_irq(chip, what, limit=4000):
    for _ in range(limit):
        if await chip.bus.read(R.STATUS2) & R.S2_IRQ:
            await chip.bus.write(R.STATUS2, R.S2_IRQ)
            return
    raise AssertionError(f"no IRQ: {what}")


# ------------------------------------------------------------------- UART
@cocotb.test()
async def test_uart_odd_b(dut):
    for B, N, S in ((35, 5, 5), (417, 9, 46)):
        chip = await fresh(dut, pe_fw.uart_ex(B=B, N=N, K=1, S=S, bits=8, parity=pe_fw.PAR_EVEN).words(),
                           R.owner_map(p0=R.OWN_TX))
        chip.loop.append((0, 1))
        await chip.bus.start()
        await ClockCycles(dut.clk, 3 * B)
        t0 = len(chip.samples)
        data = [0x55, 0xA3, 0x00, 0xFF]
        assert await chip.transfer(data, n_rx=len(data), limit=40000) == data, B
        await ClockCycles(dut.clk, 2 * B)
        wire = uart.decode_ex(chip.trace(0, t0), B, 9, start=B + 1)
        assert [w & 0xFF for w in wire] == data, (B, wire)
        await chip.bus.write(R.RUN, 0)


async def send_frame(chip, byte, B, stop=1):
    for b in [0] + [(byte >> i) & 1 for i in range(8)] + [stop]:
        chip.drive[1] = b
        await ClockCycles(chip.dut.clk, B)
    chip.drive[1] = 1
    await ClockCycles(chip.dut.clk, 2 * B)


@cocotb.test()
async def test_uart_break_rx(dut):
    B = 64
    chip = await fresh(dut, pe_fw.uart_ex(B=B, N=5, K=1, S=8).words(), R.owner_map(p0=R.OWN_TX))
    chip.drive[1] = 1
    await chip.bus.dmem_write(2, [0])
    await chip.bus.start()
    await ClockCycles(dut.clk, 3 * B)
    await send_frame(chip, 0x00, B)                    # a real zero byte
    assert await chip.read_rx(1) == [0x00]
    await send_frame(chip, 0x5A, B, stop=0)            # framing error with data: not a break
    assert not await chip.bus.read(R.STATUS2) & R.S2_IRQ
    chip.drive[1] = 0                                  # a break: 20 bit times low
    await ClockCycles(dut.clk, 20 * B)
    chip.drive[1] = 1
    await ClockCycles(dut.clk, 3 * B)
    await wait_irq(chip, "break")
    await chip.bus.write(R.RUN, 0)
    assert await chip.bus.dmem_read(2, 1) == [1], "one break"
    await chip.bus.write(R.RUN, 1)
    await send_frame(chip, 0xC3, B)
    assert await chip.read_rx(1) == [0xC3], "frames work after a break"
    await chip.bus.write(R.RUN, 0)


@cocotb.test()
async def test_uart_break_tx(dut):
    B = 64
    n = 15
    chip = await fresh(dut, pe_fw.uart_ex(B=B, N=5, K=1, S=8).words(), R.owner_map(p0=R.OWN_TX))
    chip.loop.append((0, 1))
    await chip.bus.dmem_write(2, [0, n])
    await chip.bus.start()
    await ClockCycles(dut.clk, 3 * B)
    t0 = len(chip.samples)
    await chip.bus.write(R.DOORBELL, 0)
    await ClockCycles(dut.clk, (n + 3) * B)
    tr = chip.trace(0, t0)
    low = tr.count(0)
    assert n * B <= low <= n * B + 8, f"break of {low} cycles"
    assert await chip.transfer([0x3C], n_rx=1, limit=20000) == [0x3C]
    await chip.bus.write(R.RUN, 0)
    assert await chip.bus.dmem_read(2, 1) == [1], "its own break is received"


# -------------------------------------------------------------------- SPI
def sck_periods(chip, t0):
    """Cycles between SCK edges while CS is low."""
    e, prev = [], None
    for t, s in enumerate(chip.samples[t0:]):
        lv, cs = (s >> SCK) & 1, (s >> CS) & 1
        if not cs and prev is not None and lv != prev:
            e.append(t)
        prev = lv
    return [y - x for x, y in zip(e, e[1:])]


async def stream(dut, B, n, mode, msb=1):
    cpol, cpha = mode >> 1, mode & 1
    chip = await fresh(dut, pe_fw.spi_master_stream(B=B, cpol=cpol, cpha=cpha, msb=msb).words(),
                       R.owner_map(p2=R.OWN_TX, p3=R.OWN_TX))
    up = [random.getrandbits(8) for _ in range(n)]
    dev = spi.SpiSlave(SCK, MOSI, MISO, CS, cpol, cpha, msb, tx=list(up))
    chip.devices.append(dev)
    await chip.bus.start()
    await ClockCycles(dut.clk, 40)
    down = [random.getrandbits(8) for _ in range(n)]
    await chip.bus.dmem_write(0, [n] + down)
    t0 = len(chip.samples)
    await chip.bus.write(R.DOORBELL, 0)
    await wait_irq(chip, f"stream B={B} mode {mode}")
    assert await chip.bus.dmem_read(0x7F, 1) == [0], f"B={B}: firmware fell behind"
    assert dev.rx == down, f"B={B} mode {mode}: device got {dev.rx}"
    assert await chip.bus.dmem_read(0x80, n) == up, f"B={B} mode {mode}: chip read"
    p = sck_periods(chip, t0)
    assert len(p) == 2 * 8 * n - 1, f"{len(p)} SCK periods"
    H = B // 2
    assert set(p) <= {H, B - H}, f"B={B} mode {mode}: SCK paused: {sorted(set(p))}"
    await chip.bus.write(R.RUN, 0)


@cocotb.test()
async def test_spi_stream(dut):
    for mode in range(4):
        await stream(dut, 4, 40, mode, msb=mode & 1)


@cocotb.test()
async def test_spi_stream_fast(dut):
    for mode in range(4):
        await stream(dut, 2, 9, mode)


async def no_contention(dut, dev, pin):
    while True:
        await FallingEdge(dut.clk)
        assert not (dev.drive is not None and (int(dut.uio_oe.value) >> pin) & 1), "both sides drive DIO"


@cocotb.test()
async def test_spi_3wire_master(dut):
    for mode in range(4):
        cpol, cpha = mode >> 1, mode & 1
        chip = await fresh(dut, pe_fw.spi_master_3wire(B=32, cpol=cpol, cpha=cpha).words(),
                           R.owner_map(p2=R.OWN_TX, p3=R.OWN_TX))
        back = [random.getrandbits(8) for _ in range(3)]
        dev = spi.Spi3Slave(SCK, MOSI, CS, cpol, cpha, tx=list(back))
        chip.devices.append(dev)
        t = cocotb.start_soon(no_contention(dut, dev, MOSI))
        await chip.bus.start()
        await ClockCycles(dut.clk, 40)
        await chip.bus.dmem_write(0, [3, 0, 0x11, 0x22, 0x33])
        await chip.bus.write(R.DOORBELL, 0)
        await wait_irq(chip, "3-wire write")
        assert dev.rx == [0x11, 0x22, 0x33], f"mode {mode}: {dev.rx}"
        dev.rx.clear()
        await chip.bus.dmem_write(0, [1, 3, 0x81])
        await chip.bus.write(R.DOORBELL, 0)
        await wait_irq(chip, "3-wire read")
        assert dev.rx == [0x81], f"mode {mode}: {dev.rx}"
        assert await chip.bus.dmem_read(0x80, 3) == back, f"mode {mode}: read"
        t.cancel()
        await chip.bus.write(R.RUN, 0)


@cocotb.test()
async def test_spi_3wire_slave(dut):
    for mode in range(4):
        cpol, cpha = mode >> 1, mode & 1
        chip = await fresh(dut, pe_fw.spi_slave_3wire(cpol=cpol, cpha=cpha).words(), R.owner_map(p3=R.OWN_TX))
        m = spi.SpiMasterBench(chip, miso=None, cpol=cpol, cpha=cpha, half=24, setup=40)
        back = [random.getrandbits(8) for _ in range(3)]
        await chip.bus.set_addr(R.TXF)
        for d in back:
            await chip.bus.write_same(d)
        await chip.bus.start()
        await ClockCycles(dut.clk, 40)
        await m.frame([0x12, 0x34, 0x56])
        assert await chip.read_rx(3) == [0x12, 0x34, 0x56], f"mode {mode}: write"
        assert await m.frame([0x81], n_read=3) == back, f"mode {mode}: read"
        assert await chip.read_rx(1) == [0x81]
        await chip.bus.write(R.RUN, 0)


@cocotb.test()
async def test_spi_slave_words(dut):
    for bits in (9, 12, 15, 16):
        for mode in range(4):
            cpol, cpha = mode >> 1, mode & 1
            msb = bits & 1
            chip = await fresh(dut, pe_fw.spi_slave_words(bits, cpol, cpha, msb).words(), R.owner_map(p4=R.OWN_TX))
            m = spi.SpiMasterBench(chip, cpol=cpol, cpha=cpha, msb=msb, bits=bits, half=12)
            n = 4
            mask = (1 << bits) - 1
            out = [random.getrandbits(bits) for _ in range(n)]
            back = [random.getrandbits(bits) for _ in range(n)]
            await chip.bus.set_addr(R.TXF)
            for w in back[:3]:                     # 8 FIFO bytes at most: top up below
                await chip.bus.write_same(w & 0xFF)
                await chip.bus.write_same(w >> 8)
            await chip.bus.start()
            await ClockCycles(dut.clk, 40)
            await chip.bus.write(R.TXF, back[3] & 0xFF)
            await chip.bus.write(R.TXF, back[3] >> 8)
            got = await m.frame(out)
            tag = f"{bits} bits mode {mode} msb {msb}"
            assert got == [w & mask for w in back], f"{tag}: master read {got}, sent {back}"
            rx = await chip.read_rx(2 * n)
            words = [rx[2 * i] | rx[2 * i + 1] << 8 for i in range(n)]
            assert words == out, f"{tag}: slave read {words}, sent {out}"
            await chip.bus.write(R.RUN, 0)


# -------------------------------------------------------------------- I2C
SCL, SDA = 6, 7


async def i2c_master_chip(dut, peer=None, **kw):
    chip = await fresh(dut, pe_fw.i2c_master(scl=SCL, sda=SDA, **kw).words(),
                       R.owner_map(p6=R.OWN_TX, p7=R.OWN_RX), od=0xC0)
    if peer is not None:
        chip.devices.append(peer)
    await chip.bus.start()
    await ClockCycles(dut.clk, 40)
    return chip


async def i2c_req(chip, addr, wdata=(), nread=0, hi=None):
    if hi is not None:
        await chip.bus.dmem_write(0x7E, [hi])
    await chip.bus.dmem_write(0, [addr, len(wdata), nread] + list(wdata))
    await chip.bus.write(R.DOORBELL, 0)
    await wait_irq(chip, "I2C master", limit=30000)
    return (await chip.bus.dmem_read(0x7F, 1))[0], (await chip.bus.dmem_read(0x80, nread) if nread else [])


@cocotb.test()
async def test_i2c_10bit_master(dut):
    A = 0x2A5
    dev = i2c.I2cSlave(SCL, SDA, addr10=A)
    chip = await i2c_master_chip(dut, dev, B=125, addr10=True)
    data = [random.getrandbits(8) for _ in range(3)]
    st, _ = await i2c_req(chip, A & 0xFF, [0x20] + data, hi=A >> 8)
    assert st == 0 and dev.mem[0x20:0x23] == data, (st, dev.mem[0x20:0x23])
    assert dev.seen[0] == 0xF0 | (A >> 7 & 6), [hex(x) for x in dev.seen]
    dev.mem[0x30:0x33] = [1, 2, 3]
    st, got = await i2c_req(chip, A & 0xFF, [0x30], 3, hi=A >> 8)
    assert st == 0 and got == [1, 2, 3], (st, got)
    st, _ = await i2c_req(chip, 0x5A, [0x00], hi=A >> 8)
    assert st == 1, "second address byte is wrong: NACK"
    st, _ = await i2c_req(chip, A & 0xFF, [0x00], hi=(A >> 8) ^ 1)
    assert st == 1, "first address byte is wrong: NACK"
    await chip.bus.write(R.RUN, 0)


@cocotb.test()
async def test_i2c_hs_master(dut):
    dev = i2c.I2cSlave(SCL, SDA, 0x50)
    chip = await i2c_master_chip(dut, dev, B=125, hs_B=14, sdly=2)
    data = [random.getrandbits(8) for _ in range(4)]
    t0 = len(chip.samples)
    st, _ = await i2c_req(chip, 0x50, [0x10] + data)
    assert st == 0 and dev.mem[0x10:0x14] == data, (st, dev.mem[0x10:0x14])
    assert dev.seen[0] == 0x08, "the master code comes first"
    assert dev.starts == 2, "START, master code, repeated START"
    # the master code bits run at F/S speed, and the address and data bits at HS speed
    tr = chip.trace(SCL, t0)
    rises = [i for i in range(1, len(tr)) if tr[i] and not tr[i - 1]]
    gaps = [y - x for x, y in zip(rises, rises[1:])]
    assert max(gaps[:7]) >= 120, f"master code: {gaps[:7]}"
    assert min(gaps[9:15]) <= 22, f"data at HS speed (14 + 2 of stretch check): {gaps[9:15]}"
    st, got = await i2c_req(chip, 0x50, [0x10], 4)
    assert st == 0 and got == data, (st, got)
    await chip.bus.write(R.RUN, 0)


class Rival:
    """A second master on the bus: after a START it drives `byte`, one bit per SCL fall, wired-AND."""

    def __init__(self, byte):
        self.byte, self.bit, self.sda_o, self.prev_scl, self.prev_sda = byte, 8, 1, 1, 1
        self.active = False

    def step(self, line, t):
        scl, sda = (line >> SCL) & 1, (line >> SDA) & 1
        if self.prev_scl and scl and self.prev_sda and not sda and not self.active:   # START
            self.active, self.bit = True, 8
        elif self.active and self.prev_scl and not scl:
            self.bit -= 1
            if self.bit >= 0:
                self.sda_o = (self.byte >> self.bit) & 1
            else:
                self.sda_o, self.active = 1, False
        self.prev_scl, self.prev_sda = scl, sda
        bus = sda & self.sda_o
        return (line & ~(1 << SDA)) | (bus << SDA)


@cocotb.test()
async def test_i2c_arbitration(dut):
    # The chip sends 1011000 W, the rival 1010000 W: the chip reads 0 after sending 1 at bit 4
    rival = Rival(0xA0)
    chip = await i2c_master_chip(dut, rival, B=64, arb=True)
    await chip.bus.dmem_write(0, [0x58, 1, 0, 0x55])
    await chip.bus.write(R.DOORBELL, 0)
    t0 = len(chip.samples)
    lost_at = None
    for _ in range(4000):
        await FallingEdge(dut.clk)
        if lost_at is None and rival.bit < 4:             # the bit that decides has been driven
            lost_at = rival.bit
        if rival.bit <= 2 and rival.active:               # from the next bit on the chip is off the bus
            assert not int(dut.uio_oe.value) & (1 << SDA), f"the loser drives SDA at rival bit {rival.bit}"
        if not rival.active and lost_at is not None:
            break
    await wait_irq(chip, "arbitration")
    assert await chip.bus.dmem_read(0x7F, 1) == [3], "the chip lost"
    assert not int(dut.uio_oe.value) & 0xC0, "both lines released"
    tr_scl, tr_sda = chip.trace(SCL, t0), chip.trace(SDA, t0)
    stops = sum(1 for k in range(1, len(tr_sda)) if tr_scl[k] and tr_scl[k - 1] and tr_sda[k] and not tr_sda[k - 1])
    assert stops == 0, "the loser sends no STOP"
    await chip.bus.write(R.RUN, 0)

    # The rival sends 1s (it would have lost), so the chip wins and finishes its transfer
    dev = i2c.I2cSlave(SCL, SDA, 0x28)
    chip = await i2c_master_chip(dut, dev, B=64, arb=True)
    chip.devices.append(Rival(0xFF))
    st, _ = await i2c_req(chip, 0x28, [0x20, 0x5A])
    assert st == 0 and dev.mem[0x20] == 0x5A, (st, dev.mem[0x20])
    await chip.bus.write(R.RUN, 0)

    # The bus is busy when the request comes: the chip waits for it to be free
    chip = await i2c_master_chip(dut, i2c.I2cSlave(SCL, SDA, 0x28), B=64, arb=True)
    chip.drive[SDA] = 0
    await chip.bus.dmem_write(0, [0x28, 1, 0, 0x01])
    await chip.bus.write(R.DOORBELL, 0)
    await ClockCycles(dut.clk, 300)
    assert not int(dut.uio_oe.value) & 0xC0, "no START on a busy bus"
    del chip.drive[SDA]
    await wait_irq(chip, "free bus")
    assert await chip.bus.dmem_read(0x7F, 1) == [0]
    await chip.bus.write(R.RUN, 0)


async def i2c_slave_chip(dut, h=60, **kw):
    chip = await fresh(dut, pe_fw.i2c_slave(scl=SCL, sda=SDA, **kw).words(), R.owner_map(p7=R.OWN_TX), od=0xC0)
    m = i2c.I2cMaster(chip, SCL, SDA, h=h)
    chip.devices.append(m)
    await chip.bus.start()
    await ClockCycles(dut.clk, 40)
    return chip, m


@cocotb.test()
async def test_i2c_10bit_slave(dut):
    A = 0x1C7
    chip, m = await i2c_slave_chip(dut, addr10=A)
    first = 0xF0 | ((A >> 7) & 6)
    data = [random.getrandbits(8) for _ in range(3)]
    await m.start()
    acks = [await m.wbyte(first), await m.wbyte(A & 0xFF), await m.wbyte(0x40)]
    acks += [await m.wbyte(d) for d in data]
    await m.stop()
    assert all(acks), acks
    await ClockCycles(dut.clk, 30)
    await chip.bus.write(R.RUN, 0)
    assert await chip.bus.dmem_read(0x40, 3) == data
    await chip.bus.write(R.RUN, 1)
    # read: address write, pointer, repeated START with the first byte and R
    await m.start()
    acks = [await m.wbyte(first), await m.wbyte(A & 0xFF), await m.wbyte(0x40)]
    await m.start()
    acks.append(await m.wbyte(first | 1))
    got = [await m.rbyte(k < 2) for k in range(3)]
    await m.stop()
    assert all(acks) and got == data, (acks, got)
    # other addresses
    await m.start()
    ok = await m.wbyte(first ^ 2)
    await m.stop()
    assert not ok, "another first byte"
    await m.start()
    acks = [await m.wbyte(first), await m.wbyte((A & 0xFF) ^ 1)]
    await m.stop()
    assert acks == [True, False], acks
    await m.start()
    ok = await m.wbyte(first | 1)
    await m.stop()
    assert not ok, "a read needs a matching write before it"
    await chip.bus.write(R.RUN, 0)


@cocotb.test()
async def test_i2c_slave_stretch(dut):
    hold = 400
    chip, m = await i2c_slave_chip(dut, h=40, addr=0x42, hold=hold)
    t0 = len(chip.samples)
    acks = await m.write(0x42, [0x10, 0xA5, 0x5A])
    assert all(acks), acks
    tr = chip.trace(SCL, t0)
    # longest SCL low: the master's own low phase is under 100 cycles
    run = best = 0
    for lv in tr:
        run = run + 1 if not lv else 0
        best = max(best, run)
    assert best >= hold, f"SCL low for {best} cycles at most"
    await ClockCycles(dut.clk, 500)
    await chip.bus.write(R.RUN, 0)
    assert await chip.bus.dmem_read(0x10, 2) == [0xA5, 0x5A]


@cocotb.test()
async def test_i2c_general_call(dut):
    chip, m = await i2c_slave_chip(dut, general_call=True)
    await m.start()
    acks = [await m.wbyte(0x00), await m.wbyte(0x06), await m.wbyte(0x77)]
    await m.stop()
    assert all(acks), acks
    await wait_irq(chip, "general call")
    await ClockCycles(dut.clk, 40)
    await chip.bus.write(R.RUN, 0)
    assert await chip.bus.dmem_read(pe_fw.I2C_GC_MEM, 2) == [0x06, 0x77]
    await chip.bus.write(R.RUN, 1)
    # its own address still works
    acks = await m.write(0x42, [0x08, 0x11])
    assert all(acks), acks
    await chip.bus.write(R.RUN, 0)


# -------------------------------------------------------------------- USB
@cocotb.test()
async def test_usb_device_suspend(dut):
    F = pe_fw
    for speed in ("FS", "LS"):
        chip, peer, B, jp, kp = await usb_device_chip(dut, speed)
        assert not await chip.bus.read(R.CSTAT) & R.CS_ETX, "just started"
        await ClockCycles(dut.clk, F.USB_IDLE + 200)
        assert await chip.bus.read(R.CSTAT) & R.CS_ETX, f"{speed}: idle bus"
        # a SOF is activity: the flag clears, and the timer runs again
        await send_token_data(chip, peer, B, [(tok(F.PID_SOF, 0x12, 1), {})])
        await ClockCycles(dut.clk, 40 * B)
        assert not await chip.bus.read(R.CSTAT) & R.CS_ETX, f"{speed}: a SOF is not idle"
        await ClockCycles(dut.clk, F.USB_IDLE + 200)
        assert await chip.bus.read(R.CSTAT) & R.CS_ETX, f"{speed}: idle again"
        # resume: K for a while, seen on PIN_IN, then an EOP
        peer.send_levels([usb.K] * 40 + [usb.SE0] * 2 + [usb.J], B)
        await ClockCycles(dut.clk, 20 * B)
        assert not (await chip.bus.read(R.PIN_IN) >> jp) & 1, f"{speed}: K visible on PIN_IN"
        await peer.idle()
        await ClockCycles(dut.clk, 40 * B)
        assert not await chip.bus.read(R.CSTAT) & R.CS_ETX, f"{speed}: awake"
        # the device still works
        data = [random.getrandbits(8) for _ in range(4)]
        await send_token_data(chip, peer, B, [(tok(F.PID_OUT), {}), (usb.data_packet(F.PID_DATA0, data), {})])
        await wait_irq(chip, f"{speed}: OUT after resume")
        await chip.bus.write(R.RUN, 0)
        assert await chip.bus.dmem_read(F.UD_RXDATA + 1, 4) == data, speed


@cocotb.test()
async def test_usb_host_resume(dut):
    F = pe_fw
    for speed in ("FS", "LS"):
        chip, peer, B, jp, kp = await usb_host_chip(dut, speed)
        n = 2
        await chip.bus.dmem_write(F.UH_OP, [3])
        await chip.bus.dmem_write(F.UH_N, [n])
        t0 = len(chip.samples)
        await chip.bus.write(R.DOORBELL, 0)
        await wait_irq(chip, f"{speed}: resume")
        jt, kt = chip.trace(jp, t0), chip.trace(kp, t0)
        k = sum(1 for a, b in zip(jt, kt) if not a and b)
        se0 = sum(1 for a, b in zip(jt, kt) if not a and not b)
        assert n * 252 + B <= k <= n * 262 + B, f"{speed}: K for {k} cycles"
        assert 2 * B - 2 <= se0 <= 2 * B + 2, f"{speed}: EOP of {se0} cycles"
        order = [(a, b) for a, b in zip(jt, kt)]
        states = [x for i, x in enumerate(order) if i == 0 or order[i - 1] != x]
        assert states == [(1, 0), (0, 1), (0, 0), (1, 0)], f"{speed}: J, K, SE0, J with no glitch: {states}"
        assert not int(dut.uio_oe.value) & ((1 << jp) | (1 << kp)), "released after the resume"
        await chip.bus.write(R.RUN, 0)

