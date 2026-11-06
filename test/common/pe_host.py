# SPDX-License-Identifier: Apache-2.0
"""
pe_host: host-bus driver and register map (v2) for top-level tests.

What this file does
-------------------
* Register addresses, mirroring the header of src/host/pe_regs.v.
* `HostBus`: drives the 4-bit host bus on ui_in / uo_out the way the RP2040
  will (set op + data, toggle strobe, wait for the ack toggle). On top of
  that it provides byte `write`/`read`, `load_program` (IMEM staging +
  commit with auto-increment), `start` (RUN/RESTART) and DMEM block access.
  A read of an empty RX FIFO returns None (the RTL drives X on the data
  nibble).

Pins: ui[3:0] data, ui[5:4] op, ui[6] strobe; uo[3:0] data out, uo[4] ack,
uo[5] irq.
"""

from cocotb.triggers import ClockCycles

OP_ADDR, OP_WRITE, OP_READ, OP_SYNC = range(4)

ID, VERSION, RUN, RESTART, STATUS, ERR_CLR, IRQ_EN, FIFO_CNT = range(8)
TXF, RXF, IMEM_ADDR, PIN_IN, PIN_IN_H, IMEM_ADDR_H, STATUS2, DOORBELL = range(8, 16)
IMEM_DATA, PIN_OD, PIN_FILT, PIN_OWNER = 0x10, 0x14, 0x16, 0x18
START_PC, PC, CSTAT = 0x20, 0x22, 0x24
DMEM_ADDR, DMEM_DATA = 0x28, 0x29
IMEM_WORDS = 160         # IMEM_DEPTH of tt_um_sahil_protoemu.v

# STATUS bits
ST_TXF_EMPTY, ST_TXF_FULL, ST_RXF_EMPTY, ST_RXF_FULL = 0x01, 0x02, 0x04, 0x08
ST_TXF_OVR, ST_RXF_OVR, ST_OWNER_ERR, ST_IMEM_ERR = 0x10, 0x20, 0x40, 0x80
# STATUS2 bits
S2_ILL, S2_STK, S2_RX_OVR, S2_TX_UNDER, S2_IRQ = 0x01, 0x02, 0x04, 0x08, 0x10
# CSTAT bits
CS_RUN, CS_BLOCKED, CS_Z, CS_C, CS_TO, CS_ETX, CS_ERX, CS_DB = (1 << i for i in range(8))

# pin owners
OWN_CTRL, OWN_TX, OWN_RX = 0, 1, 2


async def usb_result(bus, n=3):
    """pe_fw.usb_host: the first byte received, UH_STATUS, UH_RXLEN."""
    return ([(await bus.dmem_read(0x80, 1))[0]] + await bus.dmem_read(7, 2))[:n]


def owner_map(**pins):
    """owner_map(p0=OWN_TX, p3=OWN_RX) -> 16-bit PIN_OWNER value."""
    v = 0
    for k, o in pins.items():
        v |= o << (2 * int(k[1:]))
    return v


class HostBus:
    def __init__(self, dut, timeout=50, prefix=""):
        self.dut = dut
        self.timeout = timeout
        self.stb = 0
        self.extra = 0  # ui_in[7], unused
        self.ui = getattr(dut, prefix + "ui_in")
        self.uo = getattr(dut, prefix + "uo_out")

    def _uo_bit(self, i):
        return int(str(self.uo.value)[7 - i])

    def _uo_nibble(self):
        """uo_out[3:0], or None if it holds X (reading an empty RX FIFO)."""
        low = str(self.uo.value)[4:]
        return int(low, 2) if set(low) <= {"0", "1"} else None

    def irq(self):
        return self._uo_bit(5)

    async def op(self, op, nibble=0):
        ack = self._uo_bit(4)
        self.stb ^= 1
        self.ui.value = (self.extra << 7) | (self.stb << 6) | (op << 4) | (nibble & 0xF)
        for _ in range(self.timeout):
            await ClockCycles(self.dut.clk, 1)
            if self._uo_bit(4) != ack:
                return self._uo_nibble()
        raise AssertionError("host bus: no ack")

    async def set_addr(self, addr):
        await self.op(OP_ADDR, addr >> 4)
        await self.op(OP_ADDR, addr & 0xF)

    async def write(self, addr, data):
        await self.set_addr(addr)
        await self.op(OP_WRITE, data >> 4)
        await self.op(OP_WRITE, data & 0xF)

    async def write_same(self, data):
        """Write to the current address again (FIFO / IMEM / DMEM data)."""
        await self.op(OP_WRITE, data >> 4)
        await self.op(OP_WRITE, data & 0xF)

    async def read(self, addr):
        await self.set_addr(addr)
        return await self.read_same()

    async def read_same(self):
        hi = await self.op(OP_READ)
        lo = await self.op(OP_READ)
        return None if hi is None or lo is None else (hi << 4) | lo

    async def write16(self, addr, v):
        await self.write(addr, v & 0xFF)
        await self.write(addr + 1, v >> 8)

    async def read16(self, addr):
        return await self.read(addr) | (await self.read(addr + 1) << 8)

    async def load_program(self, words, start=0):
        assert start + len(words) <= IMEM_WORDS, f"program of {len(words)} words does not fit IMEM"
        await self.write(IMEM_ADDR, start & 0xFF)
        await self.write(IMEM_ADDR_H, start >> 8)
        for w in words:
            for b in range(4):
                await self.write(IMEM_DATA + b, (w >> (8 * b)) & 0xFF)

    async def owners(self, v):
        for b in range(4):
            await self.write(PIN_OWNER + b, (v >> (8 * b)) & 0xFF)

    async def start(self, pc=0):
        await self.write(RUN, 0)
        await self.write16(START_PC, pc)
        await self.write(RESTART, 1)
        await self.write(RUN, 1)

    async def dmem_write(self, addr, data):
        await self.write(DMEM_ADDR, addr)
        await self.set_addr(DMEM_DATA)
        for d in data:
            await self.write_same(d)

    async def dmem_read(self, addr, n):
        await self.write(DMEM_ADDR, addr)
        await self.set_addr(DMEM_DATA)
        return [await self.read_same() for _ in range(n)]
