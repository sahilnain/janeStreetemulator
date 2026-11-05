# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_crc: unit test for src/unit/pe_crc.v (CRC unit).

What is checked
---------------
1. usb_crc16: bits from RX and from TX give the CRC usb.py computes independently.
2. usb_crc5: the top-aligned register gives USB CRC5 and the residual 0x6000.
3. residual16: packet + CRC gives 0x800D.
4. skip: SKIP bits do not change the register; INIT reloads the count; SKIP up to 255.
5. source: only the selected source counts; SRC 0 counts nothing.
6. append: app_ok follows APPEND and `sent`; INIT clears `sent`.
7. random_model: random POLY, INIT, SKIP, bits against a Python model.
"""

import random

import cocotb
import pe_isa as I
import usb
from tbutil import fall, settle, start, v

INS = ("cfg_we", "cfg_idx", "cfg_data", "rd_idx", "tx_v", "tx_bit", "rx_v", "rx_bit", "sent_set")


async def cfg(dut, idx, data):
    dut.cfg_we.value, dut.cfg_idx.value, dut.cfg_data.value = 1, idx, data
    await fall(dut)
    dut.cfg_we.value = 0


async def init(dut):
    for s in INS:
        getattr(dut, s).value = 0
    await start(dut)


async def setup(dut, src, poly, ini, skip=0, app=0):
    await cfg(dut, I.CRC_CTRL, I.crc_ctrl(src, app, skip))
    await cfg(dut, I.CRC_POLY, poly)
    await cfg(dut, I.CRC_INIT, ini)


async def feed(dut, bits, src=I.CRC_RX):
    for b in bits:
        if src == I.CRC_TX:
            dut.tx_v.value, dut.tx_bit.value = 1, b
        else:
            dut.rx_v.value, dut.rx_bit.value = 1, b
        await fall(dut)
    dut.tx_v.value = dut.rx_v.value = 0


async def value(dut):
    dut.rd_idx.value = 0
    await settle()
    return v(dut.rd_data)


def bits_of(data):
    return [(d >> i) & 1 for d in data for i in range(8)]


@cocotb.test()
async def test_usb_crc16(dut):
    await init(dut)
    for src in (I.CRC_RX, I.CRC_TX):
        for data in ([], [0], [0xFF] * 8, [random.getrandbits(8) for _ in range(20)]):
            await setup(dut, src, 0x8005, 0xFFFF)
            await feed(dut, bits_of(data), src)
            await settle()
            c = v(dut.value)
            lo, hi = usb.crc16(data)
            wire = lo | hi << 8
            exp = sum(((wire >> i) & 1) << (15 - i) for i in range(16))
            assert c == exp, (src, data, hex(c), hex(exp))


@cocotb.test()
async def test_usb_crc5(dut):
    await init(dut)
    for _ in range(40):
        bits = [random.getrandbits(1) for _ in range(11)]
        await setup(dut, I.CRC_RX, 0x2800, 0xF800)
        await feed(dut, bits)
        await settle()
        wire = usb.crc5(bits)
        c = v(dut.value)
        assert [(c >> (15 - i)) & 1 for i in range(5)] == wire
        await feed(dut, wire)
        assert await value(dut) == 0x6000


@cocotb.test()
async def test_residual16(dut):
    await init(dut)
    for n in (0, 1, 8, 64):
        data = [random.getrandbits(8) for _ in range(n)]
        await setup(dut, I.CRC_RX, 0x8005, 0xFFFF)
        await feed(dut, bits_of(data + usb.crc16(data)))
        assert await value(dut) == 0x800D, n


@cocotb.test()
async def test_skip(dut):
    await init(dut)
    await setup(dut, I.CRC_RX, 0x8005, 0xFFFF, skip=8)
    await feed(dut, bits_of([0xC3]))
    assert await value(dut) == 0xFFFF
    await feed(dut, [1])
    assert await value(dut) != 0xFFFF
    await cfg(dut, I.CRC_INIT, 0x1234)
    await feed(dut, bits_of([0x77]))
    assert await value(dut) == 0x1234


@cocotb.test()
async def test_skip_large(dut):
    """All SKIP bits count: the register holds for exactly SKIP bits, up to 255."""
    await init(dut)
    for skip in (0x20, 0x5A, 0xA5, 0xFF):
        await setup(dut, I.CRC_RX, 0x8005, 0xFFFF, skip=skip)
        await feed(dut, [random.getrandbits(1) for _ in range(skip)])
        assert await value(dut) == 0xFFFF, skip
        await feed(dut, [1])
        assert await value(dut) != 0xFFFF, skip


@cocotb.test()
async def test_source(dut):
    await init(dut)
    await setup(dut, I.CRC_RX, 0x8005, 0xFFFF)
    await feed(dut, [1, 0, 1], I.CRC_TX)
    assert await value(dut) == 0xFFFF
    await setup(dut, I.CRC_TX, 0x8005, 0xFFFF)
    await feed(dut, [1, 0, 1], I.CRC_RX)
    assert await value(dut) == 0xFFFF
    await setup(dut, I.CRC_OFF, 0x8005, 0xFFFF)
    await feed(dut, [1, 0, 1], I.CRC_TX)
    await feed(dut, [1, 0, 1], I.CRC_RX)
    assert await value(dut) == 0xFFFF


@cocotb.test()
async def test_append(dut):
    await init(dut)
    await setup(dut, I.CRC_TX, 0x8005, 0xFFFF, app=0)
    await settle()
    assert not v(dut.app_ok)
    await setup(dut, I.CRC_TX, 0x8005, 0xFFFF, app=1)
    await settle()
    assert v(dut.app_ok)
    dut.sent_set.value = 1
    await fall(dut)
    dut.sent_set.value = 0
    await settle()
    assert not v(dut.app_ok)
    dut.rd_idx.value = 1
    await settle()
    assert v(dut.rd_data) == 1
    await cfg(dut, I.CRC_INIT, 0)
    await settle()
    assert v(dut.app_ok) and v(dut.rd_data) == 0
    dut.rd_idx.value = 5
    await settle()
    assert v(dut.rd_data) == 0


@cocotb.test()
async def test_random_model(dut):
    await init(dut)
    for _ in range(100):
        poly, ini, skip = random.getrandbits(16), random.getrandbits(16), random.randint(0, 5)
        await setup(dut, I.CRC_RX, poly, ini, skip=skip)
        r = ini
        bits = [random.getrandbits(1) for _ in range(random.randint(0, 24))]
        for k, b in enumerate(bits):
            if k >= skip:
                fb = ((r >> 15) & 1) ^ b
                r = (r << 1) & 0xFFFF
                if fb:
                    r ^= poly
        await feed(dut, bits)
        assert await value(dut) == r
