# SPDX-License-Identifier: Apache-2.0
"""
usb: USB line and packet model for the tests.

What this file does
-------------------
* `crc5(bits)`, `crc16(data)`, `token(pid, addr, ep)`, `data_packet(pid, data)`:
  packets as lists of bytes, computed here independently of the RTL.
* `encode(packet, sync_bits=8)`: the bus levels of one packet, one per bit:
  sync, bit stuffing, NRZI, then SE0, SE0, J.
* `UsbPeer`: the other end of the bus, in Chip.devices. It drives J / K / SE0 per
  cycle and fails the test if it and the chip drive the bus together. A bit
  may last a fractional number of cycles.
* `decode(samples, jp, kp, B)`: the packets the chip sent, from a per-cycle
  trace. Checks: all levels on the bit grid, sync pattern, stuff bits, whole
  bytes, SE0 for exactly 2 bits, then J.
"""

from cocotb.triggers import FallingEdge

J, K, SE0 = "J", "K", "0"


def crc5(bits):
    r = 0x1F
    for b in bits:
        fb = ((r >> 4) & 1) ^ b
        r = (r << 1) & 0x1F
        if fb:
            r ^= 0x05
    return [((~r) >> (4 - i)) & 1 for i in range(5)]       # bits in wire order


def crc16(data):
    r = 0xFFFF
    for d in data:
        for i in range(8):
            fb = ((r >> 15) & 1) ^ ((d >> i) & 1)
            r = (r << 1) & 0xFFFF
            if fb:
                r ^= 0x8005
    v = ~r & 0xFFFF
    w = sum(((v >> (15 - i)) & 1) << i for i in range(16))  # wire order, LSB-first bytes
    return [w & 0xFF, w >> 8]


def token(pid, addr, ep):
    bits = [(addr >> i) & 1 for i in range(7)] + [(ep >> i) & 1 for i in range(4)]
    bits += crc5(bits)
    return [pid, sum(b << i for i, b in enumerate(bits[:8])), sum(b << i for i, b in enumerate(bits[8:]))]


def data_packet(pid, data):
    return [pid] + list(data) + crc16(data)


def encode(packet, sync_bits=8, stuff=True, extra_one=None):
    """Levels, one per bit. sync_bits < 8: bits lost in a hub. extra_one: bit
    index after which the stuffing is skipped once (error on purpose)."""
    bits = [0] * (sync_bits - 1) + [1]
    for b in packet:
        bits += [(b >> i) & 1 for i in range(8)]
    out, level, ones, n = [], J, 0, 0
    for b in bits:
        level = level if b else (K if level == J else J)
        out.append(level)
        ones = ones + 1 if b else 0
        if ones == 6:
            if stuff and n != extra_one:
                level = K if level == J else J
                out.append(level)
            ones = 0
        n += 1
    return out + [SE0, SE0, J]


class UsbPeer:
    def __init__(self, chip, jp, kp):
        self.chip, self.jp, self.kp = chip, jp, kp
        self.q = []
        self.ends = []                                     # sample index when each send ended
        self.pending_end = False

    def step(self, line, t):
        if not self.q:
            if self.pending_end:
                self.ends.append(t)
                self.pending_end = False
            return line
        oe = int(self.chip.sig("uio_oe").value)
        assert not oe & ((1 << self.jp) | (1 << self.kp)), f"bus contention at sample {t}"
        lv = self.q.pop(0)
        v = {J: (1, 0), K: (0, 1), SE0: (0, 0)}[lv]
        line &= ~((1 << self.jp) | (1 << self.kp))
        return line | (v[0] << self.jp) | (v[1] << self.kp)

    def send_levels(self, levels, bit):
        acc = 0.0
        for lv in levels:
            acc += bit
            n = int(acc)
            acc -= n
            self.q += [lv] * n
        self.pending_end = True

    def send(self, packet, bit, **kw):
        self.send_levels(encode(packet, **kw), bit)

    async def idle(self):
        clk = self.chip.dut.clk
        while self.q or self.pending_end:
            await FallingEdge(clk)


def _level(s, jp, kp):
    j, k = (s >> jp) & 1, (s >> kp) & 1
    assert not (j and k), "SE1 on the bus"
    return J if j else (K if k else SE0)


def decode(samples, jp, kp, B):
    """All packets in `samples`, each as dict(data, start, se0, end)."""
    n = len(samples)
    lv = [_level(s, jp, kp) for s in samples]
    i, pkts = 0, []
    while i < n:
        if lv[i] != K:                                     # idle, or a stray SE0 (bus reset)
            i += 1
            continue
        start = i
        levels = []
        while True:
            assert i + B <= n, f"packet at {start} runs off the trace"
            seg = lv[i:i + B]
            assert len(set(seg)) == 1, f"level change inside a bit at {i}, packet at {start}: {seg}"
            if seg[0] == SE0:
                break
            levels.append(seg[0])
            i += B
        se0 = i
        for t in range(2 * B):
            assert lv[se0 + t] == SE0, f"SE0 too short: {t} cycles, packet at {start}"
        assert lv[se0 + 2 * B] == J, "no J after SE0: SE0 too long"
        prev, bits = J, []
        for x in levels:
            bits.append(1 if x == prev else 0)
            prev = x
        assert bits[:8] == [0] * 7 + [1], f"bad sync {bits[:8]}"
        data_bits, ones = [], 1
        k = 8
        while k < len(bits):
            b = bits[k]
            if ones == 6:
                assert b == 0, f"stuff bit missing at bit {k}"
                ones = 0
                k += 1
                continue
            data_bits.append(b)
            ones = ones + 1 if b else 0
            k += 1
        assert len(data_bits) % 8 == 0, f"{len(data_bits)} bits, not whole bytes"
        data = [sum(data_bits[m + q] << q for q in range(8)) for m in range(0, len(data_bits), 8)]
        pkts.append(dict(data=data, start=start, se0=se0, end=se0 + 3 * B))
        i = se0 + 2 * B
    return pkts
