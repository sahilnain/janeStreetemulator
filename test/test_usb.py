# SPDX-License-Identifier: Apache-2.0
"""
test_usb.py: USB at both speeds, chip as device and as host (run through test/tb.v).

What this file does
-------------------
usb.UsbPeer is the other end of the bus. Packets are built and checked by usb.py,
independently of the RTL: stuffing, NRZI, CRC, sync and EOP on the wire, every
level on the bit grid, and no bus contention. Bit times in cycles: full speed
B = 4, low speed B = 32 (48 MHz clock).

Device tests
1. test_dev_out          OUT + DATA: data stored, ACK within 2 .. 7.5 bit times.
2. test_dev_setup        SETUP + DATA0, endpoint recorded.
3. test_dev_stuffing     all 1s and worst-case patterns across byte boundaries.
4. test_dev_handshakes   NAK and STALL chosen by the host side.
5. test_dev_in           IN: data with the right CRC16; ACK seen by firmware.
6. test_dev_in_zlp       zero-length IN packet.
7. test_dev_in_nak       IN with no data ready: NAK.
8. test_dev_in_no_ack    the host does not answer: reported, data kept.
9. test_dev_ignores      wrong address, SOF: silence, then normal operation.
Errors on purpose
10. test_dev_bad_crc     bad data CRC, bad token CRC: no handshake.
11. test_dev_stuff_error a missing stuff bit: no handshake.
12. test_dev_short_sync  sync shortened to 4 bits (lost in hubs).
13. test_dev_clock_error the peer's bit time is 33.33 / 4.17 cycles or 3% off.
14. test_dev_reset       bus reset is reported; short SE0 is not.
Host tests
15. test_host_out        token + DATA + ACK from the peer.
16. test_host_in         token, DATA from the peer, chip answers ACK.
17. test_host_timeout    no answer: UH_TIMEOUT.
18. test_host_bad_crc    corrupt DATA: UH_BADPKT, no ACK.
19. test_host_reset      bus reset length; keep-alive EOP.
"""

import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "common"))

import cocotb  # noqa: E402
import pe_fw as F  # noqa: E402
import pe_host as R  # noqa: E402
import usb  # noqa: E402
from chip import Chip  # noqa: E402
from cocotb.triggers import ClockCycles  # noqa: E402

SPEEDS = {"FS": (4, 0, 1), "LS": (32, 1, 0)}   # B, jp (high in J), kp; D+ = P0, D- = P1
ADDR, EP = 0x2A, 3


def tok(pid, addr=ADDR, ep=EP):
    return usb.token(pid, addr, ep)


async def device(dut, speed, inlen=0, inpid=F.PID_DATA1, indata=(), ouths=F.PID_ACK, inhs=F.PID_NAK):
    B, jp, kp = SPEEDS[speed]
    chip = Chip(dut)
    await chip.reset()
    chip.ext = 0xFF & ~(1 << kp)
    peer = usb.UsbPeer(chip, jp, kp)
    chip.devices.append(peer)
    words = F.usb_device(B=B, jp=jp, kp=kp).words()
    await chip.bus.load_program(words)
    await chip.bus.owners(R.owner_map(**{f"p{jp}": R.OWN_TX, f"p{kp}": R.OWN_TX}))
    await chip.bus.dmem_write(F.UD_ADDR, [ADDR, 0, 0, 0, inlen, inpid, ouths, inhs, 0])
    if indata:
        await chip.bus.dmem_write(F.UD_INDATA, list(indata))
    await chip.bus.start()
    await ClockCycles(dut.clk, 150)
    return chip, peer, B, jp, kp


async def send_token_data(chip, peer, B, pkts, gap=2, bit=None):
    """Packets with gap bit times of idle between them."""
    bit = bit or B
    t0 = len(chip.samples)
    for i, (p, kw) in enumerate(pkts):
        peer.send(p, bit, **kw)
        await peer.idle()
        if i + 1 < len(pkts):
            await ClockCycles(chip.dut.clk, int(gap * bit))
    return t0


async def irq(chip, limit=4000):
    for _ in range(limit):
        if await chip.bus.read(R.STATUS2) & R.S2_IRQ:
            await chip.bus.write(R.STATUS2, R.S2_IRQ)
            return True
    return False


async def stopped(chip):
    await chip.bus.write(R.RUN, 0)


def answer(chip, t0, jp, kp, B, peer_end):
    """The chip's packets after the peer stopped, and the turnaround in bits,
    measured as in the USB spec from the SE0 -> J edge (the peer ends one J bit later)."""
    pk = usb.decode(chip.samples[peer_end:], jp, kp, B)
    return pk, (pk[0]["start"] / B + 1 if pk else None)


@cocotb.test()
async def test_dev_out(dut):
    for speed in SPEEDS:
        chip, peer, B, jp, kp = await device(dut, speed)
        data = [random.getrandbits(8) for _ in range(8)]
        ends0 = len(peer.ends)
        await send_token_data(chip, peer, B, [(tok(F.PID_OUT), {}), (usb.data_packet(F.PID_DATA0, data), {})])
        assert await irq(chip), speed
        await ClockCycles(dut.clk, 20 * B)
        pk, turn = answer(chip, 0, jp, kp, B, peer.ends[ends0 + 1])
        assert pk and pk[0]["data"] == [F.PID_ACK], f"{speed}: {pk}"
        assert 2 <= turn <= 7.5, f"{speed}: turnaround {turn} bits"
        await stopped(chip)
        m = await chip.bus.dmem_read(0, 9)
        assert m[F.UD_EVENT] == F.EV_OUT and m[F.UD_TOKEN] == F.PID_OUT and m[F.UD_EP] == EP
        assert m[F.UD_RXLEN] == len(data) + 3, m
        got = await chip.bus.dmem_read(F.UD_RXDATA, len(data) + 1)
        assert got == [F.PID_DATA0] + data, speed
        dut._log.info(f"{speed}: ACK {turn} bit times after EOP")


@cocotb.test()
async def test_dev_setup(dut):
    for speed in SPEEDS:
        chip, peer, B, jp, kp = await device(dut, speed)
        setup = [0x80, 6, 0, 1, 0, 0, 18, 0]
        await send_token_data(chip, peer, B, [(tok(F.PID_SETUP, ep=0), {}), (usb.data_packet(F.PID_DATA0, setup), {})])
        assert await irq(chip)
        await stopped(chip)
        m = await chip.bus.dmem_read(0, 9)
        assert m[F.UD_TOKEN] == F.PID_SETUP and m[F.UD_EP] == 0
        assert await chip.bus.dmem_read(F.UD_RXDATA + 1, 8) == setup


@cocotb.test()
async def test_dev_stuffing(dut):
    for speed in SPEEDS:
        for data in ([0xFF] * 8, [0x7E, 0xFC, 0x3F, 0xFF, 0x00, 0xFF], [0x00] * 8, [0xFF]):
            chip, peer, B, jp, kp = await device(dut, speed, inlen=len(data) + 1, indata=data)
            await send_token_data(chip, peer, B, [(tok(F.PID_OUT), {}), (usb.data_packet(F.PID_DATA0, data), {})])
            assert await irq(chip), (speed, data)
            await ClockCycles(dut.clk, 15 * B)
            assert await chip.bus.dmem_read(F.UD_RXDATA + 1, len(data)) == data
            # the same data back through the device: stuffing checked by usb.decode
            ends0 = len(peer.ends)
            await send_token_data(chip, peer, B, [(tok(F.PID_IN), {})])
            await ClockCycles(dut.clk, (len(data) * 10 + 60) * B)
            pk, _ = answer(chip, 0, jp, kp, B, peer.ends[ends0])
            assert pk[0]["data"] == usb.data_packet(F.PID_DATA1, data), (speed, data, pk)
            await stopped(chip)


@cocotb.test()
async def test_dev_handshakes(dut):
    for speed in SPEEDS:
        for hs in (F.PID_NAK, F.PID_STALL):
            chip, peer, B, jp, kp = await device(dut, speed, ouths=hs)
            ends0 = len(peer.ends)
            await send_token_data(chip, peer, B, [(tok(F.PID_OUT), {}), (usb.data_packet(F.PID_DATA1, [1, 2, 3]), {})])
            assert await irq(chip)
            await ClockCycles(dut.clk, 15 * B)
            pk, _ = answer(chip, 0, jp, kp, B, peer.ends[ends0 + 1])
            assert pk[0]["data"] == [hs]
            await stopped(chip)


async def in_txn(dut, speed, data, inhs=F.PID_NAK, ack=True):
    chip, peer, B, jp, kp = await device(dut, speed, inlen=(len(data) + 1) if data is not None else 0,
                                         indata=data or (), inhs=inhs)
    ends0 = len(peer.ends)
    await send_token_data(chip, peer, B, [(tok(F.PID_IN), {})])
    n = 60 + 12 * len(data or ())
    for _ in range(n * B):
        await ClockCycles(dut.clk, 1)
        if chip.samples and (chip.samples[-1] >> jp) & 1 == 0 and (chip.samples[-1] >> kp) & 1 == 0 \
                and len(chip.samples) > peer.ends[ends0] + 3 * B:
            break
    await ClockCycles(dut.clk, 5 * B)
    pk, turn = answer(chip, 0, jp, kp, B, peer.ends[ends0])
    assert pk, f"{speed}: no answer"
    assert 2 <= turn <= 7.5, f"{speed}: turnaround {turn} bits"
    return chip, peer, B, pk, ack, turn


@cocotb.test()
async def test_dev_in(dut):
    for speed in SPEEDS:
        for n in (1, 8, 64 if speed == "FS" else 8):
            data = [random.getrandbits(8) for _ in range(n)]
            chip, peer, B, pk, _, turn = await in_txn(dut, speed, data)
            assert pk[0]["data"] == usb.data_packet(F.PID_DATA1, data), f"{speed} n={n}: {pk[0]}"
            peer.send([F.PID_ACK], B)
            await peer.idle()
            assert await irq(chip), f"{speed} n={n}"
            await stopped(chip)
            m = await chip.bus.dmem_read(0, 9)
            assert m[F.UD_EVENT] == F.EV_IN_ACK and m[F.UD_INLEN] == 0, m
            dut._log.info(f"{speed} n={n}: DATA {turn} bit times after EOP")


@cocotb.test()
async def test_dev_in_zlp(dut):
    for speed in SPEEDS:
        chip, peer, B, pk, _, _ = await in_txn(dut, speed, [])
        assert pk[0]["data"] == [F.PID_DATA1, 0x00, 0x00], pk
        await stopped(chip)


@cocotb.test()
async def test_dev_in_nak(dut):
    for speed in SPEEDS:
        for hs in (F.PID_NAK, F.PID_STALL):
            chip, peer, B, pk, _, _ = await in_txn(dut, speed, None, inhs=hs)
            assert pk[0]["data"] == [hs]
            await stopped(chip)


@cocotb.test()
async def test_dev_in_no_ack(dut):
    for speed in SPEEDS:
        data = [1, 2, 3]
        chip, peer, B, pk, _, _ = await in_txn(dut, speed, data)
        assert await irq(chip, 200 * B)
        await stopped(chip)
        m = await chip.bus.dmem_read(0, 9)
        assert m[F.UD_EVENT] == F.EV_IN_NOACK and m[F.UD_INLEN] == 4, m


async def silence(chip, peer, jp, kp, B, end_idx, cycles):
    await ClockCycles(chip.dut.clk, cycles)
    pk = usb.decode(chip.samples[peer.ends[end_idx]:], jp, kp, B)
    assert not pk, f"unexpected answer {pk}"


@cocotb.test()
async def test_dev_ignores(dut):
    for speed in SPEEDS:
        chip, peer, B, jp, kp = await device(dut, speed)
        await send_token_data(chip, peer, B, [(tok(F.PID_IN, addr=ADDR ^ 1), {})])
        await silence(chip, peer, jp, kp, B, 0, 20 * B)
        await send_token_data(chip, peer, B, [(tok(F.PID_OUT, addr=ADDR ^ 1), {}),
                                              (usb.data_packet(F.PID_DATA0, [9, 9]), {})])
        await silence(chip, peer, jp, kp, B, 2, 20 * B)
        await send_token_data(chip, peer, B, [(usb.token(F.PID_SOF, 0x12, 5), {})])
        await silence(chip, peer, jp, kp, B, 3, 20 * B)
        ends0 = len(peer.ends)
        await send_token_data(chip, peer, B, [(tok(F.PID_OUT), {}), (usb.data_packet(F.PID_DATA0, [7]), {})])
        assert await irq(chip), speed
        await ClockCycles(dut.clk, 15 * B)
        assert answer(chip, 0, jp, kp, B, peer.ends[ends0 + 1])[0][0]["data"] == [F.PID_ACK]
        await stopped(chip)


@cocotb.test()
async def test_dev_bad_crc(dut):
    for speed in SPEEDS:
        chip, peer, B, jp, kp = await device(dut, speed)
        bad = usb.data_packet(F.PID_DATA0, [1, 2, 3, 4])
        bad[2] ^= 0x10
        await send_token_data(chip, peer, B, [(tok(F.PID_OUT), {}), (bad, {})])
        await silence(chip, peer, jp, kp, B, 1, 25 * B)
        assert not await chip.bus.read(R.STATUS2) & R.S2_IRQ
        t = tok(F.PID_IN)
        t[2] ^= 0x80
        await send_token_data(chip, peer, B, [(t, {})])
        await silence(chip, peer, jp, kp, B, 2, 25 * B)
        await stopped(chip)


@cocotb.test()
async def test_dev_stuff_error(dut):
    for speed in SPEEDS:
        chip, peer, B, jp, kp = await device(dut, speed)
        data = [0xFF, 0xFF, 0x12]
        await send_token_data(chip, peer, B, [(tok(F.PID_OUT), {}),
                                              (usb.data_packet(F.PID_DATA0, data), dict(extra_one=19))])
        await silence(chip, peer, jp, kp, B, 1, 25 * B)
        await stopped(chip)


@cocotb.test()
async def test_dev_short_sync(dut):
    for speed in SPEEDS:
        chip, peer, B, jp, kp = await device(dut, speed)
        data = [0xA5, 0x5A]
        await send_token_data(chip, peer, B, [(tok(F.PID_OUT), dict(sync_bits=4)),
                                              (usb.data_packet(F.PID_DATA0, data), dict(sync_bits=4))], gap=6)
        assert await irq(chip), speed
        await stopped(chip)
        assert await chip.bus.dmem_read(F.UD_RXDATA + 1, 2) == data


@cocotb.test()
async def test_dev_clock_error(dut):
    for speed, bits in (("FS", (4.1667, 3.88, 4.12)), ("LS", (33.33, 31.04, 32.96))):
        for bit in bits:
            chip, peer, B, jp, kp = await device(dut, speed)
            data = [random.getrandbits(8) for _ in range(8)] + [0xFF, 0xFF]
            await send_token_data(chip, peer, B, [(tok(F.PID_OUT), {}), (usb.data_packet(F.PID_DATA0, data), {})],
                                  bit=bit)
            assert await irq(chip), (speed, bit)
            await stopped(chip)
            assert await chip.bus.dmem_read(F.UD_RXDATA + 1, len(data)) == data, (speed, bit)


@cocotb.test()
async def test_dev_reset(dut):
    for speed in SPEEDS:
        chip, peer, B, jp, kp = await device(dut, speed)
        peer.send_levels([usb.SE0] * 2, B)                  # short: keep-alive
        await peer.idle()
        peer.send_levels([usb.J], B)
        await peer.idle()
        await ClockCycles(dut.clk, 300)
        assert not await chip.bus.read(R.STATUS2) & R.S2_IRQ, "EOP reported as reset"
        peer.send_levels([usb.SE0] * 40, 16)                # 13 us
        await peer.idle()
        peer.send_levels([usb.J] * 4, B)
        await peer.idle()
        assert await irq(chip), speed
        await ClockCycles(dut.clk, 300)
        await send_token_data(chip, peer, B, [(tok(F.PID_OUT), {}), (usb.data_packet(F.PID_DATA0, [5]), {})])
        assert await irq(chip), f"{speed}: not alive after reset"
        await stopped(chip)


async def host(dut, speed):
    B, jp, kp = SPEEDS[speed]
    chip = Chip(dut)
    await chip.reset()
    chip.ext = 0xFF & ~(1 << kp)
    peer = usb.UsbPeer(chip, jp, kp)
    chip.devices.append(peer)
    await chip.bus.load_program(F.usb_host(B=B, jp=jp, kp=kp).words())
    await chip.bus.owners(R.owner_map(**{f"p{jp}": R.OWN_TX, f"p{kp}": R.OWN_TX}))
    await chip.bus.start()
    await ClockCycles(dut.clk, 60)
    return chip, peer, B, jp, kp


async def go(chip, t, dpid=0, data=(), op=0):
    await chip.bus.write(R.RUN, 1)
    await chip.bus.dmem_write(0, [op] + list(t) + [dpid, len(data)])
    if data:
        await chip.bus.dmem_write(0x10, list(data))
    t0 = len(chip.samples)
    await chip.bus.write(R.DOORBELL, 0)
    return t0


async def wait_tx(chip, jp, kp, B, t0, n=1, limit=40000):
    """Run until n packets from the chip are complete after t0."""
    for _ in range(limit // 8):
        await ClockCycles(chip.dut.clk, 8)
        seg = chip.samples[t0:]
        c = sum(1 for i in range(1, len(seg))
                if (seg[i] >> jp) & 1 == 1 and (seg[i - 1] >> jp) & 1 == 0 and (seg[i - 1] >> kp) & 1 == 0)
        if c >= n:
            await ClockCycles(chip.dut.clk, 2 * B)
            return usb.decode(chip.samples[t0:], jp, kp, B)
    raise AssertionError(f"no packet after {t0}")


@cocotb.test()
async def test_host_out(dut):
    for speed in SPEEDS:
        for pid, tk in ((F.PID_OUT, tok(F.PID_OUT)), (F.PID_SETUP, tok(F.PID_SETUP, ep=0))):
            chip, peer, B, jp, kp = await host(dut, speed)
            data = [random.getrandbits(8) for _ in range(8)] + [0xFF, 0xFF]
            t0 = await go(chip, tk, F.PID_DATA0, data)
            pk = await wait_tx(chip, jp, kp, B, t0, 2)
            assert pk[0]["data"] == tk, f"{speed}: token {pk[0]}"
            assert pk[1]["data"] == usb.data_packet(F.PID_DATA0, data), f"{speed}: data {pk[1]}"
            assert pk[1]["start"] - pk[0]["end"] >= 2 * B, "inter-packet gap too short"
            peer.send([F.PID_ACK], B)
            await peer.idle()
            assert await irq(chip), speed
            await stopped(chip)
            r = await chip.bus.dmem_read(F.UH_RPID, 3)
            assert r == [F.PID_ACK, F.UH_OK, 1], r


@cocotb.test()
async def test_host_in(dut):
    for speed in SPEEDS:
        for data in ([1, 2, 3, 4, 5, 6, 7, 8], [], [0xFF] * 4):
            chip, peer, B, jp, kp = await host(dut, speed)
            tk = tok(F.PID_IN)
            t0 = await go(chip, tk)
            pk = await wait_tx(chip, jp, kp, B, t0, 1)
            assert pk[0]["data"] == tk
            await ClockCycles(dut.clk, 3 * B)
            peer.send(usb.data_packet(F.PID_DATA1, data), B)
            await peer.idle()
            e = peer.ends[-1]
            pk = await wait_tx(chip, jp, kp, B, e, 1)
            assert pk[0]["data"] == [F.PID_ACK], f"{speed}: {pk}"
            assert pk[0]["start"] + B >= 2 * B, "ACK too early"
            assert await irq(chip), speed
            await stopped(chip)
            r = await chip.bus.dmem_read(F.UH_RPID, 3)
            assert r == [F.PID_DATA1, F.UH_OK, len(data) + 3], r
            assert await chip.bus.dmem_read(0x81, len(data)) == data if data else True


@cocotb.test()
async def test_host_timeout(dut):
    for speed in SPEEDS:
        chip, peer, B, jp, kp = await host(dut, speed)
        await go(chip, tok(F.PID_IN))
        assert await irq(chip), speed
        await stopped(chip)
        assert (await chip.bus.dmem_read(F.UH_STATUS, 1))[0] == F.UH_TIMEOUT


@cocotb.test()
async def test_host_bad_crc(dut):
    for speed in SPEEDS:
        chip, peer, B, jp, kp = await host(dut, speed)
        t0 = await go(chip, tok(F.PID_IN))
        await wait_tx(chip, jp, kp, B, t0, 1)
        await ClockCycles(dut.clk, 3 * B)
        bad = usb.data_packet(F.PID_DATA1, [1, 2, 3])
        bad[1] ^= 4
        peer.send(bad, B)
        await peer.idle()
        assert await irq(chip), speed
        await ClockCycles(dut.clk, 15 * B)
        assert not usb.decode(chip.samples[peer.ends[-1]:], jp, kp, B), "ACK for a corrupt packet"
        await stopped(chip)
        assert (await chip.bus.dmem_read(F.UH_STATUS, 1))[0] == F.UH_BADPKT


@cocotb.test()
async def test_host_reset(dut):
    for speed in SPEEDS:
        chip, peer, B, jp, kp = await host(dut, speed)
        t0 = await go(chip, (0, 0, 0), op=1, data=[0] * 2)
        assert await irq(chip), speed
        await ClockCycles(dut.clk, 8)
        seg = [((s >> jp) & 1, (s >> kp) & 1) for s in chip.samples[t0:]]
        n = sum(1 for x in seg if x == (0, 0))
        assert 2 * 250 <= n <= 2 * 300, f"{speed}: reset SE0 {n} cycles"
        assert seg[-1] == (1, 0), "bus not back to J"
        t0 = await go(chip, (0, 0, 0), op=2)
        assert await irq(chip), speed
        await ClockCycles(dut.clk, 8)
        seg = [((s >> jp) & 1, (s >> kp) & 1) for s in chip.samples[t0:]]
        n = sum(1 for x in seg if x == (0, 0))
        assert 2 * B - 2 <= n <= 2 * B + 2, f"{speed}: keep-alive SE0 {n} cycles"
        assert seg[-1] == (1, 0)
        await stopped(chip)
