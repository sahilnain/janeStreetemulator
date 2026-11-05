# SPDX-License-Identifier: Apache-2.0
"""
spi: SPI devices for the top-level tests.

What this file does
-------------------
* `SpiSlave`: a cycle-level SPI slave attached to Chip.devices (tests the
  chip as SPI master). It watches CS / SCK / MOSI on the chip's pads and
  drives MISO with zero delay: for CPHA 0 it presents bit 0 when CS falls
  and shifts on trailing edges; for CPHA 1 it shifts on leading edges.
  It samples MOSI on the other edge. `rx` collects received bytes; bytes
  are answered from `tx` (0xFF when empty). It also records every SCK edge
  time, for clock-rate checks.
* `master_xfer(chip, ...)`: the testbench as SPI master in any mode (tests
  the chip as SPI slave): drives CS / SCK / MOSI through chip.drive with half
  period h and samples MISO on the sampling edge.
* `check_miso_edges(...)`: in the per-cycle trace of a frame, MISO may only
  change after a shift edge, as the SPI mode requires.
"""

from cocotb.triggers import ClockCycles, FallingEdge


class SpiSlave:
    def __init__(self, sck, mosi, miso, cs, cpol=0, cpha=0, msb=1, tx=(), bits=8):
        self.p = dict(sck=sck, mosi=mosi, miso=miso, cs=cs)
        self.cpol, self.cpha, self.msb, self.bits = cpol, cpha, msb, bits
        self.tx = list(tx)
        self.rx = []
        self.edges = []
        self.prev_sck = cpol
        self.prev_cs = 1
        self.out = 1
        self.cur = 0xFF
        self.nout = 0
        self.nin = 0
        self.sh = 0
        self.cs_low_cycles = 0

    def _bit(self, k):
        return (self.cur >> (self.bits - 1 - k if self.msb else k)) & 1

    def _load(self):
        self.cur = self.tx.pop(0) if self.tx else (1 << self.bits) - 1
        self.nout = 0

    def step(self, line, t):
        b = lambda n: (line >> self.p[n]) & 1  # noqa: E731
        cs, sck, mosi = b("cs"), b("sck"), b("mosi")
        if self.prev_cs and not cs:              # select
            self._load()
            self.nin, self.sh = 0, 0
            if self.cpha == 0:
                self.out = self._bit(0)
                self.nout = 1
        if not cs:
            self.cs_low_cycles += 1
            if sck != self.prev_sck:
                self.edges.append(t)
                leading = sck != self.cpol
                sample = leading if self.cpha == 0 else not leading
                if sample:
                    self.sh = (self.sh << 1 | mosi) if self.msb else (self.sh | mosi << self.nin)
                    self.nin += 1
                    if self.nin == self.bits:
                        self.rx.append(self.sh & ((1 << self.bits) - 1))
                        self.nin, self.sh = 0, 0
                else:                            # shift edge
                    if self.nout == self.bits:
                        self._load()
                    self.out = self._bit(self.nout)
                    self.nout += 1
        self.prev_cs, self.prev_sck = cs, sck
        m = 1 << self.p["miso"]
        return (line & ~m) | (self.out << self.p["miso"]) if not cs else line


def check_miso_edges(samples, sck, miso, cs, cpol, cpha):
    """While CS is low, every MISO change must follow a shift edge: the last SCK
    edge before it is the trailing edge (CPHA 0) or the leading edge (CPHA 1). In
    CPHA 1 there must be no change at all before the first leading edge."""
    b = lambda i, p: (samples[i] >> p) & 1  # noqa: E731
    last = None                                # True: last edge was a leading edge
    for i in range(1, len(samples)):
        if b(i, cs):
            last = None
            continue
        if b(i - 1, cs):                       # select cycle
            continue
        if b(i, sck) != b(i - 1, sck):
            last = b(i, sck) != cpol
        if b(i, miso) != b(i - 1, miso):
            if cpha:
                assert last is True, f"MISO changed at {i}, last SCK edge leading={last}"
            else:
                assert last is not True, f"MISO changed at {i} after a leading edge"


async def master_xfer(chip, data, sck=2, mosi=3, miso=4, cs=5, h=8, cpol=0, cpha=0, msb=1, setup=10, bits=8):
    """Testbench SPI master, any mode. Returns the bytes read on MISO."""
    clk = chip.dut.clk

    async def wait(n):
        for _ in range(n):
            await FallingEdge(clk)

    chip.drive.update({sck: cpol, mosi: 0, cs: 1})
    await wait(4)
    chip.drive[cs] = 0
    await wait(setup)
    got = []
    for d in data:
        v = 0
        for k in range(bits):
            bit = (d >> (bits - 1 - k if msb else k)) & 1
            if cpha == 0:
                chip.drive[mosi] = bit
                await wait(h)
            chip.drive[sck] = 1 - cpol                 # leading edge
            if cpha:
                chip.drive[mosi] = bit                 # CPHA 1: shift on the leading edge
                await wait(h)
                chip.drive[sck] = cpol                 # trailing edge: sample MISO
            await FallingEdge(clk)
            m = (chip.samples[-1] >> miso) & 1
            v = (v << 1 | m) if msb else (v | m << k)
            await wait(h - 1)
            if cpha == 0:
                chip.drive[sck] = cpol                 # trailing edge
        got.append(v)
    await wait(h)
    chip.drive[cs] = 1
    await wait(4)
    return got


class Spi3Slave:
    """3-wire SPI slave: first byte = command (MSB 1: read), then writes are stored in `rx`, and
    reads come from `tx`. Fails on bus contention through `line`: the chip must release DIO
    before this device drives it. Master CPOL / CPHA as for SpiSlave."""

    def __init__(self, sck, dio, cs, cpol=0, cpha=0, msb=1, tx=(), turn=8):
        self.sck, self.dio, self.cs, self.turn = sck, dio, cs, turn
        self.pending = None                 # (time, level): drive `turn` cycles after the shift edge
        self.cpol, self.cpha, self.msb = cpol, cpha, msb
        self.tx = list(tx)
        self.rx = []
        self.prev_sck, self.prev_cs = cpol, 1
        self.n, self.sh, self.reading, self.cur, self.drive = 0, 0, False, 0xFF, None
        self.nout = 0

    def _bit(self, k):
        return (self.cur >> (7 - k if self.msb else k)) & 1

    def step(self, line, t, oe=0):
        sck, cs = (line >> self.sck) & 1, (line >> self.cs) & 1
        dio = (line >> self.dio) & 1
        if not self.prev_cs and cs:
            self.drive, self.pending = None, None
        if self.prev_cs and not cs:
            self.n, self.sh, self.reading, self.nout = 0, 0, False, 0
            self.rx_cmd = None
        if not cs and sck != self.prev_sck:
            leading = sck != self.cpol
            sample = leading if self.cpha == 0 else not leading
            if self.reading:
                if not sample:
                    if self.nout == 8:
                        self.cur, self.nout = (self.tx.pop(0) if self.tx else 0xFF), 0
                    self.pending = (t + self.turn if self.drive is None else t, self._bit(self.nout))
                    self.nout += 1
            elif sample:
                self.sh = (self.sh << 1 | dio) if self.msb else (self.sh | dio << self.n)
                self.n += 1
                if self.n == 8:
                    self.rx.append(self.sh & 0xFF)
                    if self.rx[-1] & (0x80 if self.msb else 0x01) and len(self.rx) >= 1 and self.rx_cmd is None:
                        self.reading = True
                        self.cur = self.tx.pop(0) if self.tx else 0xFF
                        self.nout = 0
                    self.rx_cmd = self.rx[-1]
                    self.n, self.sh = 0, 0
        self.prev_sck, self.prev_cs = sck, cs
        if self.pending and t >= self.pending[0] and not cs:
            self.drive, self.pending = self.pending[1], None
        if self.drive is None:
            return line
        return (line & ~(1 << self.dio)) | (self.drive << self.dio)


class SpiMasterBench:
    """A testbench SPI master, for firmware slaves: drives pins through chip.drive. half = half
    the SCK period, setup = cycles from CS low to the first edge. 4-wire with miso, or
    3-wire when miso is None (mosi is the shared data pin, released while reading)."""

    def __init__(self, chip, sck=2, mosi=3, miso=4, cs=5, cpol=0, cpha=0, msb=1, bits=8, half=16, setup=32):
        self.chip, self.sck, self.mosi, self.miso, self.cs = chip, sck, mosi, miso, cs
        self.cpol, self.cpha, self.msb, self.bits, self.half, self.setup = cpol, cpha, msb, bits, half, setup
        chip.drive[cs] = 1
        chip.drive[sck] = cpol
        chip.drive[mosi] = 0

    async def _wait(self, n):
        await ClockCycles(self.chip.dut.clk, n)

    def _read_pin(self):
        return self.chip.samples[-1] >> (self.mosi if self.miso is None else self.miso) & 1

    async def word(self, value, drive=True):
        """One word out on mosi and in on miso (3-wire read: drive = False)."""
        c, n = self.chip, self.bits
        r = 0
        for k in range(n):
            i = n - 1 - k if self.msb else k
            if not self.cpha:
                if drive:
                    c.drive[self.mosi] = (value >> i) & 1
                await self._wait(self.half)
                r |= self._read_pin() << i
                c.drive[self.sck] = 1 - self.cpol
                await self._wait(self.half)
                c.drive[self.sck] = self.cpol
            else:
                c.drive[self.sck] = 1 - self.cpol
                if drive:
                    c.drive[self.mosi] = (value >> i) & 1
                await self._wait(self.half)
                r |= self._read_pin() << i
                c.drive[self.sck] = self.cpol
                await self._wait(self.half)
        return r

    async def frame(self, words, n_read=0):
        """CS low, the words, then n_read more (3-wire: the pin released), CS high."""
        c = self.chip
        c.drive[self.cs] = 0
        await self._wait(self.setup)
        got = [await self.word(w) for w in words]
        if n_read:
            if self.miso is None:
                del c.drive[self.mosi]
                await self._wait(self.half)
            got = [await self.word(0, drive=False) for _ in range(n_read)]
        await self._wait(self.half)
        c.drive[self.cs] = 1
        if self.miso is None:
            c.drive[self.mosi] = 0
        await self._wait(self.setup)
        return got

