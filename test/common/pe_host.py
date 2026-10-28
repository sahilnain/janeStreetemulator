# SPDX-License-Identifier: Apache-2.0
"""
pe_host: host-bus driver and register map for top-level tests.

What this file does
-------------------
* Register addresses, mirroring the header of src/host/pe_regs.v.
* `HostBus`: drives the 4-bit host bus on ui_in / uo_out the way the RP2040
  will (set op + data, toggle strobe, wait for the ack toggle). On top of
  that it provides byte `write`/`read`, `load_program` (IMEM staging + commit
  with auto-increment) and small config helpers. A read of an empty RX FIFO
  returns None (the RTL drives X on the data nibble).

Pins: ui[3:0] data, ui[5:4] op, ui[6] strobe; uo[3:0] data out, uo[4] ack,
uo[5] irq.
"""

from cocotb.triggers import ClockCycles

OP_ADDR, OP_WRITE, OP_READ, OP_SYNC = range(4)

ID, VERSION, RUN, RESTART, STATUS, ERR_CLR, IRQ_EN, FIFO_CNT = range(8)
TXF, RXF, IMEM_ADDR, PIN_IN = 0x08, 0x09, 0x0A, 0x0B
IMEM_DATA, PIN_OD, PIN_OWNER, CTX0 = 0x10, 0x14, 0x18, 0x20

# per-context offsets
START_PC, CFG, VOTE_N, VOTE_K, VOTE_S_LO, VOTE_S_HI, PC, CSTAT = range(8)

# STATUS bits
ST_TXF_EMPTY, ST_TXF_FULL, ST_RXF_EMPTY, ST_RXF_FULL = 0x01, 0x02, 0x04, 0x08
ST_TXF_OVR, ST_RXF_OVR, ST_OWNER_ERR, ST_IMEM_ERR = 0x10, 0x20, 0x40, 0x80


def ctx_reg(c, off):
    return CTX0 + 8 * c + off


class HostBus:
    def __init__(self, dut, timeout=50):
        self.dut = dut
        self.timeout = timeout
        self.stb = 0
        self.extra = 0  # ui_in[7], unused

    def _uo_bit(self, i):
        return int(str(self.dut.uo_out.value)[7 - i])

    def _uo_nibble(self):
        """uo_out[3:0], or None if it holds X (reading an empty RX FIFO)."""
        low = str(self.dut.uo_out.value)[4:]
        return int(low, 2) if set(low) <= {"0", "1"} else None

    async def op(self, op, nibble=0):
        dut = self.dut
        ack = self._uo_bit(4)
        self.stb ^= 1
        dut.ui_in.value = (self.extra << 7) | (self.stb << 6) | (op << 4) | (nibble & 0xF)
        for _ in range(self.timeout):
            await ClockCycles(dut.clk, 1)
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
        """Write to the current address again (FIFO / IMEM data)."""
        await self.op(OP_WRITE, data >> 4)
        await self.op(OP_WRITE, data & 0xF)

    async def read(self, addr):
        await self.set_addr(addr)
        hi = await self.op(OP_READ)
        lo = await self.op(OP_READ)
        return None if hi is None or lo is None else (hi << 4) | lo

    async def load_program(self, words, start=0, iw=24):
        nb = (iw + 7) // 8
        await self.write(IMEM_ADDR, start)
        for w in words:
            for b in range(nb):
                await self.write(IMEM_DATA + b, (w >> (8 * b)) & 0xFF)

    async def setup_ctx(self, c, start_pc=0, msb=0, vote=None):
        await self.write(ctx_reg(c, START_PC), start_pc)
        await self.write(ctx_reg(c, CFG), msb)
        if vote is not None:
            n, k, s = vote
            await self.write(ctx_reg(c, VOTE_N), n)
            await self.write(ctx_reg(c, VOTE_K), k)
            await self.write(ctx_reg(c, VOTE_S_LO), s & 0xFF)
            await self.write(ctx_reg(c, VOTE_S_HI), s >> 8)
