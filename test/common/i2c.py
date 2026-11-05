# SPDX-License-Identifier: Apache-2.0
"""
i2c: I2C devices for the top-level tests (attached to Chip.devices).

What this file does
-------------------
The bus is open drain: every cycle the line the chip sees is
    chip pads (driven low or pulled up) AND each device's outputs.
* `I2cSlave(scl, sda, addr, mem)`: a register device (tests the chip as
  master). Write: the first byte sets the register pointer, the next bytes
  are stored at pointer++. Read: returns mem[pointer++]. ACKs its address,
  NACKs other addresses, NACKs data once `nack_after` bytes were written.
  `stretch`: {event: cycles} holds SCL low after the falling edge that ends
  the address ACK ("addr") or before every read byte ("read").
  Bus checker: `starts`, `stops`, and `min_hold` = fewest cycles between an
  SCL fall and the next SDA change by the master (I2C data hold).
* `I2cMaster(scl, sda)`: the testbench as master (tests the chip as slave):
  async `write(addr, data)`, `read(addr, n)` with ACK/NACK, START/STOP,
  half period `h` cycles. Returns the ACK bits seen.
* `check_timing(samples, scl, sda, mode)`: measures a bus trace against the limits of the I2C
  standard (UM10204) for SM, FM, FM+ or HS, and returns what breaks them. Edges are ideal here, so
  a real bus also needs a margin for its rise time.
"""

from cocotb.triggers import FallingEdge


CYCLE_NS = 1000 / 48
# fscl in kHz, the others in ns. HS has no bus-free limit, as it returns to F/S mode for that.
LIMITS = {
    "SM": dict(fscl=100, low=4700, high=4000, hd_sta=4000, su_sta=4700, su_sto=4000, buf=4700, su_dat=250),
    "FM": dict(fscl=400, low=1300, high=600, hd_sta=600, su_sta=600, su_sto=600, buf=1300, su_dat=100),
    "FM+": dict(fscl=1000, low=500, high=260, hd_sta=260, su_sta=260, su_sto=260, buf=500, su_dat=50),
    "HS": dict(fscl=3400, low=160, high=60, hd_sta=160, su_sta=160, su_sto=160, buf=0, su_dat=10),
}


def starts(samples, scl, sda):
    """Indices at which a START or repeated START happens."""
    return [t for t in range(1, len(samples))
            if (samples[t - 1] >> scl) & 1 and (samples[t] >> scl) & 1
            and (samples[t - 1] >> sda) & 1 and not (samples[t] >> sda) & 1]


def check_timing(samples, scl, sda, mode, only=None):
    """The limits of `mode` broken by this trace, worst value of each, from the first START on.
    `only`: the names in LIMITS to check, when the other side is not the one being tested."""
    lim = LIMITS[mode]
    worst = {}

    def need(name, cycles):
        ns = cycles * CYCLE_NS
        if ns < lim[name] and (name not in worst or ns < worst[name]):
            worst[name] = ns

    rise = fall = sda_t = start = stop = None
    for t in range(1, len(samples)):
        pc, c = (samples[t - 1] >> scl) & 1, (samples[t] >> scl) & 1
        pd, d = (samples[t - 1] >> sda) & 1, (samples[t] >> sda) & 1
        if pc and c and pd and not d:                       # START
            if stop is not None and (rise is None or rise < stop):
                need("buf", t - stop)
            elif rise is not None:
                need("su_sta", t - rise)
            start, stop = t, None
        elif pc and c and not pd and d and start is not None:   # STOP
            if rise is not None:
                need("su_sto", t - rise)
            stop = t
        elif pd != d and not c:
            sda_t = t
        if start is None:
            continue
        if not pc and c:                                    # SCL rises
            if fall is not None:
                need("low", t - fall)
            if sda_t is not None and fall is not None and sda_t >= fall:
                need("su_dat", t - sda_t)
            if rise is not None and 48000 / (t - rise) > lim["fscl"]:
                worst["fscl"] = max(worst.get("fscl", 0), 48000 / (t - rise))
            rise = t
        elif pc and not c:                                  # SCL falls
            if rise is not None:
                need("high", t - rise)
            if start is not None and (fall is None or fall < start):
                need("hd_sta", t - start)
            fall = t
    return [f"{k} {v:.0f}, limit {lim[k]}" for k, v in worst.items() if only is None or k in only]


class I2cSlave:
    def __init__(self, scl, sda, addr=0x50, mem=None, nack_after=None, stretch=None, addr10=None):
        self.scl, self.sda, self.addr = scl, sda, addr
        self.addr10 = addr10                     # a 10-bit address replaces addr
        self.matched10 = False
        self.seen = []                           # address bytes on the bus, master codes included
        self.mem = mem if mem is not None else [0] * 256
        self.nack_after = nack_after
        self.stretch = stretch or {}
        self.sda_o = self.scl_o = 1
        self.prev_sda_o = 1
        self.master_ack = False
        self.rw = 0
        self.prev = (1, 1)
        self.state = "idle"
        self.bits = self.n = self.ptr = self.nw = 0
        self.first = True
        self.rd_byte = 0
        self.hold_left = 0
        self.starts = self.stops = 0
        self.last_fall = None
        self.min_hold = None
        self.log = []

    def _bus(self, line):
        return (line >> self.scl) & 1 & self.scl_o, (line >> self.sda) & 1 & self.sda_o

    def step(self, line, t):
        chip_sda = (line >> self.sda) & 1
        scl, sda = self._bus(line)
        pscl, psda = self.prev
        if self.hold_left:
            self.hold_left -= 1
            if not self.hold_left:
                self.scl_o = 1
        # bus checker: master data hold after SCL fall
        if (pscl == 0 and scl == 0 and sda != psda and self.sda_o == 1 and self.prev_sda_o == 1
                and self.last_fall is not None):
            d = t - self.last_fall
            self.min_hold = d if self.min_hold is None else min(self.min_hold, d)
        if pscl and scl and psda and not sda:                       # START
            self.starts += 1
            self.state, self.bits, self.n = "addr", 0, 0
            self.sda_o = 1
        elif pscl and scl and not psda and sda:                     # STOP
            self.stops += 1
            self.state = "idle"
            self.matched10 = False
            self.sda_o = 1
        elif not pscl and scl:                                      # rising: sample
            if self.state in ("addr", "addr2", "wdata"):
                self.bits = (self.bits << 1) | chip_sda & sda
                self.n += 1
            elif self.state == "rack":
                self.master_ack = sda == 0
        elif pscl and not scl:                                      # falling: drive
            self.last_fall = t
            self._falling(t)
        self.prev = (scl, sda)
        self.prev_sda_o = self.sda_o
        m = (1 << self.scl) | (1 << self.sda)
        return (line & ~m) | ((((line >> self.scl) & self.scl_o) << self.scl) |
                              (((line >> self.sda) & self.sda_o) << self.sda))

    def _falling(self, t):
        st = self.state
        if st == "ack_a10":
            self.sda_o = 1
            self.state = "addr2"
            self.bits = self.n = 0
        elif st in ("addr", "addr2", "wdata") and self.n == 8:
            byte = self.bits & 0xFF
            self.bits = self.n = 0
            if st == "addr2":                     # second byte of a 10-bit address
                if byte == self.addr10 & 0xFF:
                    self.matched10 = True
                    self.rw = 0
                    self.sda_o = 0
                    self.state = "ack_a"
                    self.first = True
                else:
                    self.state = "idle"
            elif st == "addr" and self.addr10 is not None:
                self.seen.append(byte)
                if byte & 0xFE == 0xF0 | ((self.addr10 >> 7) & 0x06):
                    if byte & 1 and self.matched10:
                        self.rw = 1
                        self.sda_o = 0
                        self.state = "ack_a"
                    elif not byte & 1:
                        self.sda_o = 0
                        self.state = "ack_a10"
                    else:
                        self.state = "idle"
                else:
                    self.state = "idle"
            elif st == "addr":
                self.seen.append(byte)
                if byte >> 1 == self.addr:
                    self.rw = byte & 1
                    self.sda_o = 0
                    self.state = "ack_a"
                    self.first = True
                else:
                    self.state = "idle"
            else:
                ok = self.nack_after is None or self.nw < self.nack_after
                if ok:
                    if self.first:
                        self.ptr = byte
                        self.first = False
                    else:
                        self.mem[self.ptr & 0xFF] = byte
                        self.ptr += 1
                    self.nw += 1
                    self.log.append(("w", byte))
                self.sda_o = 0 if ok else 1
                self.state = "ack_w" if ok else "idle"
        elif st == "ack_a":                       # end of the address ACK clock
            if self.stretch.get("addr"):
                self.scl_o = 0
                self.hold_left = self.stretch["addr"]
            if self.rw:
                self._send()
            else:
                self.sda_o = 1
                self.state = "wdata"
        elif st == "ack_w":
            self.sda_o = 1
            self.state = "wdata"
        elif st == "rdata":
            self.n += 1
            if self.n == 8:
                self.sda_o = 1                    # release for the master's ACK
                self.state = "rack"
            else:
                self.sda_o = (self.rd_byte >> (7 - self.n)) & 1
        elif st == "rack":
            if self.master_ack:
                self._send()
            else:
                self.sda_o = 1
                self.state = "idle"

    def _send(self):
        if self.stretch.get("read"):
            self.scl_o = 0
            self.hold_left = self.stretch["read"]
        self.rd_byte = self.mem[self.ptr & 0xFF]
        self.ptr += 1
        self.log.append(("r", self.rd_byte))
        self.n = 0
        self.sda_o = (self.rd_byte >> 7) & 1
        self.state = "rdata"


class I2cMaster:
    """Testbench I2C master. Attach with chip.devices.append(m); then await
    m.write(...) / m.read(...)."""

    def __init__(self, chip, scl, sda, h=60):
        self.chip, self.scl, self.sda, self.h = chip, scl, sda, h
        self.scl_o = self.sda_o = 1
        self.bus = (1, 1)

    def step(self, line, t):
        m = (1 << self.scl) | (1 << self.sda)
        out = (line & ~m) | ((((line >> self.scl) & self.scl_o) << self.scl) |
                             (((line >> self.sda) & self.sda_o) << self.sda))
        self.bus = ((out >> self.scl) & 1, (out >> self.sda) & 1)
        return out

    async def wait(self, n):
        for _ in range(n):
            await FallingEdge(self.chip.dut.clk)

    async def clock(self, sda_bit):
        """One SCL clock with SDA = sda_bit (1 = released); returns SDA at SCL high."""
        self.sda_o = sda_bit
        await self.wait(self.h // 2)
        self.scl_o = 1
        for _ in range(100000):                 # the slave may stretch
            await self.wait(1)
            if self.bus[0]:
                break
        await self.wait(self.h // 2)
        bit = self.bus[1]
        await self.wait(self.h - self.h // 2)
        self.scl_o = 0
        await self.wait(1)
        return bit

    async def start(self):
        self.sda_o = 1
        await self.wait(self.h // 2)             # a repeated START keeps the SCL low phase
        self.scl_o = 1
        await self.wait(self.h)
        self.sda_o = 0
        await self.wait(self.h)
        self.scl_o = 0
        await self.wait(2)

    async def stop(self):
        self.sda_o = 0
        await self.wait(self.h)
        self.scl_o = 1
        await self.wait(self.h)
        self.sda_o = 1
        await self.wait(self.h)

    async def wbyte(self, b):
        for k in range(8):
            await self.clock((b >> (7 - k)) & 1)
        return await self.clock(1) == 0          # ACK?

    async def rbyte(self, ack):
        v = 0
        for _ in range(8):
            v = (v << 1) | await self.clock(1)
        await self.clock(0 if ack else 1)
        return v

    async def write(self, addr, data, stop=True):
        await self.start()
        acks = [await self.wbyte(addr << 1)]
        for d in data:
            acks.append(await self.wbyte(d))
        if stop:
            await self.stop()
        return acks

    async def read(self, addr, n, ptr=None):
        acks = []
        if ptr is not None:
            acks = await self.write(addr, [ptr], stop=False)
        await self.start()
        acks.append(await self.wbyte((addr << 1) | 1))
        got = [await self.rbyte(k < n - 1) for k in range(n)] if acks[-1] else []
        await self.stop()
        return acks, got
