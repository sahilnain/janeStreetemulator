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
"""

from cocotb.triggers import FallingEdge


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
