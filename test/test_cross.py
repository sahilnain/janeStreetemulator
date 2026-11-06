# SPDX-License-Identifier: Apache-2.0
"""
test_cross.py: the Python model and the RTL talk a protocol to each other (run through test/tb.v).

What this file does
-------------------
The RTL and the model (common/model_chip.py) run on one clock with their protocol pins wired
together, and each has its own host bus. Every test runs twice, with the roles swapped: RTL
as transmitter or master with the model as receiver or slave, then the other way round. The
wiring and transaction helpers are shared with test_dual.py.

Tests (each both ways)
----------------------
1. test_uart_*        8N1 both directions at once; 1.8% baud difference; 9-bit even parity
                      with 2 stop bits, and 7-bit odd parity inverted, through pe_fw.uart_ex.
2. test_spi_*         master and slave, modes 0-3, MSB and LSB first, full duplex; 3-wire on one
                      shared data pin, no contention; a gap-free master stream.
3. test_i2c_*         master and slave on a wired-AND bus: write, repeated-START read, an address
                      that nobody answers; 10-bit addressing; two masters, one loses arbitration.
4. test_usb_*         host and device on one bus, low and full speed: SETUP, OUT, IN, NAK,
                      wrong address; contention on the bus fails the test; idle flag and resume.
"""

import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "common"))

import cocotb  # noqa: E402
import i2c  # noqa: E402
import pe_fw  # noqa: E402
import pe_host as R  # noqa: E402
import usb  # noqa: E402
from chip import Chip  # noqa: E402
from cocotb.clock import Clock  # noqa: E402
from cocotb.triggers import ClockCycles, Combine, FallingEdge  # noqa: E402
from model_chip import ModelChip  # noqa: E402
from test_dual import pads, pads_spi, pads_usb, usb_txn, usb_wait, vote  # noqa: E402

SCL, SDA = 6, 7
ROLES = ("rtl_first", "model_first")


async def setup(dut, role):
    """Returns (a, b): a is the transmitter / master side of the test."""
    cocotb.start_soon(Clock(dut.clk, 20834, unit="ps").start())
    rtl = Chip(dut, "", clock=False, pads=False)
    mod = ModelChip(dut)
    await Combine(cocotb.start_soon(rtl.reset()), cocotb.start_soon(mod.reset()))
    return (rtl, mod) if role == "rtl_first" else (mod, rtl)


async def pads_i2c(a, b):
    """SCL and SDA are wired-AND with pull-ups; all other pins stay separate."""
    m = (1 << SCL) | (1 << SDA)
    while True:
        await FallingEdge(a.dut.clk)
        la, lb = a.line(), b.line()
        both = la & lb & m
        ia = (la & ~m) | both
        ib = (lb & ~m) | both
        a.samples.append(ia)
        b.samples.append(ib)
        a.sig("uio_in").value = ia
        b.sig("uio_in").value = ib


# ------------------------------------------------------------------- UART
async def start_ex(chip, **kw):
    await chip.bus.write(R.RUN, 0)
    await chip.bus.load_program(pe_fw.uart_ex(**kw).words())
    await chip.bus.owners(R.owner_map(p0=R.OWN_TX))
    await chip.bus.start()


async def exchange(a, b, wa, wb):
    ta = cocotb.start_soon(a.transfer(wa, n_rx=len(wb)))
    tb = cocotb.start_soon(b.transfer(wb, n_rx=len(wa)))
    await Combine(ta, tb)
    assert ta.result() == wb, f"a got {ta.result()}, b sent {wb}"
    assert tb.result() == wa, f"b got {tb.result()}, a sent {wa}"


async def uart_8n1(dut, role):
    a, b = await setup(dut, role)
    cocotb.start_soon(pads(a, b))
    await Combine(cocotb.start_soon(a.start_uart(B=112, **vote(112))), cocotb.start_soon(b.start_uart(B=112, **vote(112))))
    await exchange(a, b, [random.getrandbits(8) for _ in range(10)], [random.getrandbits(8) for _ in range(10)])


async def uart_baud(dut, role):
    a, b = await setup(dut, role)
    cocotb.start_soon(pads(a, b))
    await Combine(cocotb.start_soon(a.start_uart(B=111, **vote(111))), cocotb.start_soon(b.start_uart(B=113, **vote(113))))
    await exchange(a, b, [random.getrandbits(8) for _ in range(6)], [random.getrandbits(8) for _ in range(6)])


async def uart_frames(dut, role):
    a, b = await setup(dut, role)
    cocotb.start_soon(pads(a, b))
    kw = dict(B=112, N=9, K=1, S=11)
    await Combine(cocotb.start_soon(start_ex(a, bits=9, parity=pe_fw.PAR_EVEN, stop_b2=4, **kw)),
                  cocotb.start_soon(start_ex(b, bits=9, parity=pe_fw.PAR_EVEN, stop_b2=4, **kw)))
    await ClockCycles(dut.clk, 40)
    words = [[w & 0xFF, w >> 8] for w in (random.getrandbits(9) for _ in range(4))]
    await exchange(a, b, sum(words, []), sum(reversed(words), []))


async def uart_inverted(dut, role):
    a, b = await setup(dut, role)
    # An inverted line idles low: hold the inputs low before either side starts
    for c in (a, b):
        c.ext = 0x00
    cocotb.start_soon(pads(a, b))
    kw = dict(B=112, N=9, K=1, S=11, bits=7, parity=pe_fw.PAR_ODD, stop_b2=3, inv=1)
    await Combine(cocotb.start_soon(start_ex(a, **kw)), cocotb.start_soon(start_ex(b, **kw)))
    await ClockCycles(dut.clk, 300)
    await exchange(a, b, [random.getrandbits(7) for _ in range(6)], [random.getrandbits(7) for _ in range(6)])


# -------------------------------------------------------------------- SPI
async def spi_all_modes(dut, role):
    a, b = await setup(dut, role)                  # a = master, b = slave
    cocotb.start_soon(pads_spi(a, b))
    for mode in range(4):
        for msb in (1, 0):
            cpol, cpha = mode >> 1, mode & 1
            tag = f"{role} mode {mode} msb {msb}"
            await a.bus.write(R.RUN, 0)
            await b.bus.write(R.RUN, 0)
            await b.bus.load_program(pe_fw.spi_slave(cpol=cpol, cpha=cpha, msb=msb).words())
            await b.bus.owners(R.owner_map(p4=R.OWN_TX))
            await a.bus.load_program(pe_fw.spi_master(B=16, cpol=cpol, cpha=cpha, msb=msb).words())
            await a.bus.owners(R.owner_map(p2=R.OWN_TX, p3=R.OWN_TX))
            await a.bus.start()
            await ClockCycles(dut.clk, 40)
            n = 5
            down = [random.getrandbits(8) for _ in range(n)]
            up = [random.getrandbits(8) for _ in range(n)]
            await b.bus.set_addr(R.TXF)
            for d in up + [0xEE]:
                await b.bus.write_same(d)
            await b.bus.start()
            await ClockCycles(dut.clk, 40)
            await a.bus.dmem_write(0, [n] + down)
            await a.bus.write(R.DOORBELL, 0)
            await usb_wait(a, f"{tag}: master", limit=3000)
            assert await a.bus.dmem_read(0x80, n) == up, f"{tag}: master read"
            assert await b.read_rx(n) == down, f"{tag}: slave read"


# -------------------------------------------------------------------- I2C
async def i2c_setup(dut, role, B):
    a, b = await setup(dut, role)                  # a = master, b = slave at 0x42
    cocotb.start_soon(pads_i2c(a, b))
    await a.bus.write(R.PIN_OD, 0xC0)
    await b.bus.write(R.PIN_OD, 0xC0)
    await b.bus.load_program(pe_fw.i2c_slave(0x42, SCL, SDA).words())
    await b.bus.owners(R.owner_map(p7=R.OWN_TX))
    await a.bus.load_program(pe_fw.i2c_master(B=B, scl=SCL, sda=SDA).words())
    await a.bus.owners(R.owner_map(p6=R.OWN_TX, p7=R.OWN_RX))
    await b.bus.start()
    await a.bus.start()
    await ClockCycles(dut.clk, 60)
    return a, b


async def i2c_request(a, addr, wdata=(), nread=0):
    await a.bus.dmem_write(0, [addr, len(wdata), nread] + list(wdata))
    await a.bus.write(R.DOORBELL, 0)
    await usb_wait(a, "I2C master", limit=20000)
    return (await a.bus.dmem_read(0x7F, 1))[0]


async def i2c_write_read(dut, role):
    a, b = await i2c_setup(dut, role, 126)
    data = [random.getrandbits(8) for _ in range(4)]
    assert await i2c_request(a, 0x42, [0x20] + data) == 0, "write: status"
    assert await i2c_request(a, 0x42, [0x20], 4) == 0, "read: status"
    assert await a.bus.dmem_read(0x80, 4) == data, "master reads back what it wrote through the slave"
    await b.bus.write(R.RUN, 0)
    assert await b.bus.dmem_read(0x20, 4) == data, "slave memory"
    assert not (v := i2c.check_timing(a.samples, SCL, SDA, "FM")), v


async def i2c_wrong_addr(dut, role):
    a, _ = await i2c_setup(dut, role, 126)
    assert await i2c_request(a, 0x43, [0x00]) == 1, "nobody answers: address NACK"
    assert await i2c_request(a, 0x42, [0x01, 0x5A]) == 0, "the slave still answers its own address"


async def i2c_10bit(dut, role):
    A = 0x2A5
    a, b = await setup(dut, role)
    cocotb.start_soon(pads_i2c(a, b))
    await a.bus.write(R.PIN_OD, 0xC0)
    await b.bus.write(R.PIN_OD, 0xC0)
    await b.bus.load_program(pe_fw.i2c_slave(scl=SCL, sda=SDA, addr10=A, general_call=True).words())
    await b.bus.owners(R.owner_map(p7=R.OWN_TX))
    await a.bus.load_program(pe_fw.i2c_master(B=126, scl=SCL, sda=SDA, addr10=True).words())
    await a.bus.owners(R.owner_map(p6=R.OWN_TX, p7=R.OWN_RX))
    await b.bus.start()
    await a.bus.start()
    await ClockCycles(dut.clk, 60)
    await a.bus.dmem_write(0x7E, [A >> 8])
    data = [random.getrandbits(8) for _ in range(4)]
    assert await i2c_request(a, A & 0xFF, [0x20] + data) == 0, "write: status"
    assert await i2c_request(a, A & 0xFF, [0x20], 4) == 0, "read: status"
    assert await a.bus.dmem_read(0x80, 4) == data
    assert await i2c_request(a, (A & 0xFF) ^ 1, [0x20]) == 1, "wrong low address byte"


async def i2c_two_masters(dut, role):
    """Both sides are masters and start together. Whatever happens, one wins the bus with a full
    transfer to a Python device, or the same address and data let both finish; the loser reports 3."""
    import i2c as i2c_peers
    a, b = await setup(dut, role)
    dev = i2c_peers.I2cSlave(SCL, SDA, 0x28)
    m = (1 << SCL) | (1 << SDA)

    async def wired():
        while True:
            await FallingEdge(dut.clk)
            la, lb = a.line(), b.line()
            both = dev.step(0xFF & ~m | (la & lb & m), len(a.samples)) & m
            ia, ib = (la & ~m) | both, (lb & ~m) | both
            a.samples.append(ia)
            b.samples.append(ib)
            a.sig("uio_in").value = ia
            b.sig("uio_in").value = ib

    cocotb.start_soon(wired())
    for c in (a, b):
        await c.bus.write(R.PIN_OD, 0xC0)
        await c.bus.load_program(pe_fw.i2c_master(B=64, scl=SCL, sda=SDA, arb=True).words())
        await c.bus.owners(R.owner_map(p6=R.OWN_TX, p7=R.OWN_RX))
        await c.bus.start()
    await ClockCycles(dut.clk, 60)
    # a goes to the device at 0x28 (0101000), b to 0x38 (0111000): b reads 0 after sending 1
    await a.bus.dmem_write(0, [0x28, 3, 0, 0x20, 0xA1, 0xB2])
    await b.bus.dmem_write(0, [0x38, 1, 0, 0x55])
    await Combine(cocotb.start_soon(a.bus.write(R.DOORBELL, 0)), cocotb.start_soon(b.bus.write(R.DOORBELL, 0)))
    await Combine(cocotb.start_soon(usb_wait(a, "a", limit=20000)), cocotb.start_soon(usb_wait(b, "b", limit=20000)))
    sa, sb = (await a.bus.dmem_read(0x7F, 1))[0], (await b.bus.dmem_read(0x7F, 1))[0]
    assert (sa, sb) == (0, 3), f"status a={sa}, b={sb}"
    assert dev.mem[0x20:0x22] == [0xA1, 0xB2], "the winner's data arrives intact"


# -------------------------------------------------------------------- USB
async def usb_speed(dut, role, name, B, jp, kp):
    F = pe_fw
    a, b = await setup(dut, role)                  # a = host, b = device
    cocotb.start_soon(pads_usb(a, b, jp, kp))
    own = {f"p{jp}": R.OWN_TX, f"p{kp}": R.OWN_TX}
    await a.bus.load_program(F.usb_host(B=B, jp=jp, kp=kp).words())
    await a.bus.owners(R.owner_map(**own))
    await b.bus.load_program(F.usb_device(B=B, jp=jp, kp=kp).words())
    await b.bus.owners(R.owner_map(**own))
    back = [random.getrandbits(8) for _ in range(8)]
    await b.bus.dmem_write(F.UD_ADDR, [5, 0, 0, 0, len(back) + 1, F.PID_DATA1, F.PID_ACK, F.PID_NAK, 0])
    await b.bus.dmem_write(F.UD_INDATA, back)
    await b.bus.start()
    await a.bus.start()
    await ClockCycles(dut.clk, 200)
    for pid, ep in ((F.PID_SETUP, 0), (F.PID_OUT, 2)):
        data = [random.getrandbits(8) for _ in range(8)]
        await usb_txn(a, b, usb.token(pid, 5, ep), F.PID_DATA0, data)
        await usb_wait(b, f"{name} device")
        r = await R.usb_result(a.bus, 2)
        assert r == [F.PID_ACK, F.UH_OK], f"{name}: host result {r}"
        m = await b.bus.dmem_read(0, 9)
        assert m[F.UD_EVENT] == F.EV_OUT and m[F.UD_TOKEN] == pid and m[F.UD_EP] == ep, m
        assert await b.bus.dmem_read(F.UD_RXDATA + 1, 8) == data, name
    await usb_txn(a, b, usb.token(F.PID_IN, 5, 2))
    await usb_wait(b, f"{name} device IN")
    r = await R.usb_result(a.bus, 3)
    assert r == [F.PID_DATA1, F.UH_OK, 11], f"{name}: {r}"
    assert await a.bus.dmem_read(0x81, 8) == back, name
    await ClockCycles(dut.clk, 300)
    await usb_txn(a, b, usb.token(F.PID_IN, 5, 2))
    assert await R.usb_result(a.bus, 2) == [F.PID_NAK, F.UH_OK], name
    await usb_txn(a, b, usb.token(F.PID_IN, 6, 2))
    assert (await a.bus.dmem_read(F.UH_STATUS, 1))[0] == F.UH_TIMEOUT, name


async def usb_suspend_resume(dut, role, name, B, jp, kp):
    """The host stops, the device's idle flag rises; a host resume, then a transaction works."""
    F = pe_fw
    a, b = await setup(dut, role)
    cocotb.start_soon(pads_usb(a, b, jp, kp))
    own = {f"p{jp}": R.OWN_TX, f"p{kp}": R.OWN_TX}
    await a.bus.load_program(F.usb_host(B=B, jp=jp, kp=kp).words())
    await a.bus.owners(R.owner_map(**own))
    await b.bus.load_program(F.usb_device(B=B, jp=jp, kp=kp).words())
    await b.bus.owners(R.owner_map(**own))
    await b.bus.dmem_write(F.UD_ADDR, [5, 0, 0, 0, 0, F.PID_DATA1, F.PID_ACK, F.PID_NAK, 0])
    await b.bus.start()
    await a.bus.start()
    await ClockCycles(dut.clk, F.USB_IDLE + 300)
    assert await b.bus.read(R.CSTAT) & R.CS_ETX, f"{name}: idle flag"
    await a.bus.dmem_write(F.UH_OP, [3])
    await a.bus.dmem_write(F.UH_N, [2])
    await a.bus.write(R.DOORBELL, 0)
    await usb_wait(a, f"{name}: resume")
    await ClockCycles(dut.clk, 40 * B)
    assert not await b.bus.read(R.CSTAT) & R.CS_ETX, f"{name}: awake"
    data = [random.getrandbits(8) for _ in range(4)]
    await usb_txn(a, b, usb.token(F.PID_OUT, 5, 1), F.PID_DATA0, data)
    await usb_wait(b, f"{name}: device")
    assert await R.usb_result(a.bus, 2) == [F.PID_ACK, F.UH_OK], name
    assert await b.bus.dmem_read(F.UD_RXDATA + 1, 4) == data, name


async def spi_3wire(dut, role):
    a, b = await setup(dut, role)                  # a = master, b = slave; one data pin, wired
    D = 3

    async def wired():
        while True:
            await FallingEdge(dut.clk)
            oa, ob = int(a.sig("uio_oe").value), int(b.sig("uio_oe").value)
            assert not (oa & ob & (1 << D)), "both sides drive DIO"
            la, lb = a.line(), b.line()
            d = ((la if oa >> D & 1 else 0xFF) & (lb if ob >> D & 1 else 0xFF) & (1 << D))
            ia = (la & ~(1 << D)) | d
            ib = (la & 0x24 | lb & ~0x24) & ~(1 << D) | d          # SCK, CS from the master
            a.samples.append(ia)
            b.samples.append(ib)
            a.sig("uio_in").value = ia
            b.sig("uio_in").value = ib

    cocotb.start_soon(wired())
    for mode in range(4):
        cpol, cpha = mode >> 1, mode & 1
        await a.bus.write(R.RUN, 0)
        await b.bus.write(R.RUN, 0)
        await b.bus.load_program(pe_fw.spi_slave_3wire(cpol=cpol, cpha=cpha).words())
        await b.bus.owners(R.owner_map(p3=R.OWN_TX))
        await a.bus.load_program(pe_fw.spi_master_3wire(B=48, cpol=cpol, cpha=cpha).words())
        await a.bus.owners(R.owner_map(p2=R.OWN_TX, p3=R.OWN_TX))
        await a.bus.start()
        await ClockCycles(dut.clk, 40)
        back = [random.getrandbits(8) for _ in range(3)]
        await b.bus.set_addr(R.TXF)
        for d in back:
            await b.bus.write_same(d)
        await b.bus.start()
        await ClockCycles(dut.clk, 40)
        await a.bus.dmem_write(0, [2, 0, 0x12, 0x34])
        await a.bus.write(R.DOORBELL, 0)
        await usb_wait(a, f"{role} mode {mode} write", limit=3000)
        assert await b.read_rx(2) == [0x12, 0x34], f"mode {mode}: write"
        await a.bus.dmem_write(0, [1, 3, 0x81])
        await a.bus.write(R.DOORBELL, 0)
        await usb_wait(a, f"{role} mode {mode} read", limit=3000)
        assert await a.bus.dmem_read(0x80, 3) == back, f"mode {mode}: read"
        assert await b.read_rx(1) == [0x81], "the slave reports the command"


async def spi_stream_cross(dut, role):
    a, b = await setup(dut, role)
    cocotb.start_soon(pads_spi(a, b))
    await b.bus.load_program(pe_fw.spi_slave(cpol=0, cpha=0, msb=1).words())
    await b.bus.owners(R.owner_map(p4=R.OWN_TX))
    await a.bus.load_program(pe_fw.spi_master_stream(B=16).words())
    await a.bus.owners(R.owner_map(p2=R.OWN_TX, p3=R.OWN_TX))
    await a.bus.start()
    await ClockCycles(dut.clk, 40)
    n = 6
    down = [random.getrandbits(8) for _ in range(n)]
    up = [random.getrandbits(8) for _ in range(n)]
    await b.bus.set_addr(R.TXF)
    for d in up + [0xEE]:
        await b.bus.write_same(d)
    await b.bus.start()
    await ClockCycles(dut.clk, 40)
    await a.bus.dmem_write(0, [n] + down)
    await a.bus.write(R.DOORBELL, 0)
    await usb_wait(a, f"{role}: stream", limit=3000)
    assert await a.bus.dmem_read(0x7F, 1) == [0]
    assert await a.bus.dmem_read(0x80, n) == up
    assert await b.read_rx(n) == down


def both(fn, *args):
    """Registers the scenario twice, one test per role."""
    async def _rtl(dut):
        await fn(dut, "rtl_first", *args)

    async def _mod(dut):
        await fn(dut, "model_first", *args)

    return _rtl, _mod


def _register():
    g = globals()
    for name, fn, args in (("uart_8n1", uart_8n1, ()), ("uart_baud", uart_baud, ()), ("uart_frames", uart_frames, ()),
                           ("uart_inverted", uart_inverted, ()), ("spi_all_modes", spi_all_modes, ()),
                           ("i2c_write_read", i2c_write_read, ()), ("i2c_wrong_addr", i2c_wrong_addr, ()),
                           ("i2c_10bit", i2c_10bit, ()), ("i2c_two_masters", i2c_two_masters, ()),
                           ("usb_full_speed", usb_speed, ("FS", 4, 0, 1)), ("usb_low_speed", usb_speed, ("LS", 32, 1, 0)),
                           ("usb_suspend", usb_suspend_resume, ("FS", 4, 0, 1)), ("spi_3wire", spi_3wire, ()),
                           ("spi_stream", spi_stream_cross, ())):
        r, m = both(fn, *args)
        for suffix, f in (("rtl_to_model", r), ("model_to_rtl", m)):
            f.__name__ = f.__qualname__ = f"test_{name}_{suffix}"
            g[f.__name__] = cocotb.test()(f)


_register()
