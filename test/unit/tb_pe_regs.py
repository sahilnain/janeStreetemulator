# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_regs: unit test for src/host/pe_regs.v (v2 register map).

What is checked
---------------
`Regs` models the map in docs/spec/blocks.md Section 9. Every cycle the
testbench compares every output (read data, irq, IMEM write port, FIFO and
DMEM strobes, RUN/RESTART/START_PC/DOORBELL, pin config, error clears).
1. random_access: 6000 cycles of random host reads/writes (all mapped and
   some unmapped addresses) with random status inputs and error pulses.
2. imem_commit: staging bytes + commit at 0x13, IMEM_ADDR auto-increment
   over the 8-bit boundary (IMEM_ADDR_H), commit while RUN = 1 is dropped
   and sets imem_err, ERR_CLR bit 7 clears it.
3. status2_w1c: ill/stk/underrun/ctrl_irq are sticky; W1C clears only the
   bits written; a new pulse in the same cycle wins.
4. irq_sources: each IRQ_EN bit with its source.
5. dmem_auto_inc: DMEM_DATA reads and writes increment DMEM_ADDR (wrap).
"""

import random

import cocotb
from tbutil import fall, param, settle, start, v

DEPTH = param("IMEM_DEPTH", 64)
NP = param("NUM_PINS", 8)
CW = param("FIFO_CW", 4)
AW = max(1, (DEPTH - 1).bit_length())
INS = ("r_addr", "r_wdata", "r_we", "r_re", "txf_full", "txf_empty", "txf_count", "txf_overrun",
       "rxf_rdata", "rxf_full", "rxf_empty", "rxf_count", "rxf_overrun", "pc", "cstat_in",
       "ill_err", "stk_err", "tx_underrun", "ctrl_irq", "rx_overrun", "pin_in", "owner_err",
       "dm_rdata")
OUTS = ("r_rdata", "irq", "im_we", "im_waddr", "im_wdata", "txf_wr", "txf_wdata",
        "txf_clr_overrun", "rxf_rd", "rxf_clr_overrun", "run", "restart", "start_pc", "db_set",
        "rx_ovr_clr", "pin_owner", "pin_od", "owner_clr_err", "dm_acc", "dm_we", "dm_addr",
        "dm_wdata")
MAPPED = list(range(0x00, 0x16)) + [0x18, 0x19, 0x1A, 0x1B, 0x20, 0x21, 0x22, 0x23, 0x24, 0x28, 0x29]


class Regs:
    def __init__(self):
        self.run = self.start_pc = self.owner = self.od = self.imaddr = self.stage = 0
        self.irq_en = self.imem_err = self.dm_addr = 0
        self.st2 = 0  # bits 0,1,3,4

    def comb(self, i):
        a, d, we, re = i["r_addr"], i["r_wdata"], i["r_we"], i["r_re"]
        s2 = (self.st2 & 0b11011) | (i["rx_overrun"] << 2)
        any_err = i["txf_overrun"] | i["rxf_overrun"] | i["owner_err"] | self.imem_err | int((s2 & 0xF) != 0)
        irq = int(bool((self.irq_en & 1 and not i["rxf_empty"]) or (self.irq_en & 2 and not i["txf_full"])
                       or (self.irq_en & 4 and any_err) or (self.irq_en & 8 and self.st2 & 0x10)))
        status = (self.imem_err << 7) | (i["owner_err"] << 6) | (i["rxf_overrun"] << 5) | \
                 (i["txf_overrun"] << 4) | (i["rxf_full"] << 3) | (i["rxf_empty"] << 2) | \
                 (i["txf_full"] << 1) | i["txf_empty"]
        c4 = min(CW, 4)
        cm = (1 << c4) - 1
        rd = {0x00: 0x50, 0x01: 0x02, 0x02: self.run, 0x04: status, 0x06: self.irq_en,
              0x07: (i["txf_count"] & cm) | ((i["rxf_count"] & cm) << 4), 0x09: i["rxf_rdata"],
              0x0A: self.imaddr & 0xFF, 0x0D: self.imaddr >> 8, 0x0B: i["pin_in"] & 0xFF,
              0x0C: (i["pin_in"] >> 8) & 0xFF, 0x0E: s2, 0x14: self.od & 0xFF, 0x15: self.od >> 8,
              0x18: self.owner & 0xFF, 0x19: (self.owner >> 8) & 0xFF, 0x1A: (self.owner >> 16) & 0xFF,
              0x1B: (self.owner >> 24) & 0xFF, 0x20: self.start_pc & 0xFF, 0x21: self.start_pc >> 8,
              0x22: i["pc"] & 0xFF, 0x23: i["pc"] >> 8, 0x24: (i["cstat_in"] & 0xFE) | self.run,
              0x28: self.dm_addr, 0x29: i["dm_rdata"]}.get(a, 0)
        commit = we and a == 0x13
        o = dict(r_rdata=rd, irq=irq, im_we=int(commit and not self.run), im_waddr=self.imaddr,
                 im_wdata=(d << 24) | self.stage, txf_wr=int(we and a == 0x08), txf_wdata=d,
                 txf_clr_overrun=int(we and a == 0x05 and d >> 4 & 1), rxf_rd=int(re and a == 0x09),
                 rxf_clr_overrun=int(we and a == 0x05 and d >> 5 & 1), run=self.run,
                 restart=int(we and a == 0x03 and d & 1), start_pc=self.start_pc,
                 db_set=int(we and a == 0x0F), rx_ovr_clr=int(we and a == 0x0E and d >> 2 & 1),
                 pin_owner=self.owner, pin_od=self.od, owner_clr_err=int(we and a == 0x05 and d >> 6 & 1),
                 dm_acc=int((we or re) and a == 0x29), dm_we=int(we and a == 0x29), dm_addr=self.dm_addr,
                 dm_wdata=d)
        return o

    def step(self, i, o):
        a, d, we = i["r_addr"], i["r_wdata"], i["r_we"]
        am = (1 << AW) - 1
        if we:
            if a == 0x02:
                self.run = d & 1
            if a == 0x06:
                self.irq_en = d & 0xF
            if 0x10 <= a <= 0x12:
                sh = 8 * (a - 0x10)
                self.stage = (self.stage & ~(0xFF << sh)) | (d << sh)
            if a in (0x14, 0x15):
                sh = 8 * (a - 0x14)
                self.od = ((self.od & ~(0xFF << sh)) | (d << sh)) & ((1 << NP) - 1)
            if 0x18 <= a <= 0x1B:
                sh = 8 * (a - 0x18)
                self.owner = ((self.owner & ~(0xFF << sh)) | (d << sh)) & ((1 << (2 * NP)) - 1)
            if a in (0x20, 0x21):
                sh = 8 * (a - 0x20)
                self.start_pc = ((self.start_pc & ~(0xFF << sh)) | (d << sh)) & am
        if we and a == 0x0A:
            self.imaddr = ((self.imaddr & ~0xFF) | d) & am
        elif we and a == 0x0D:
            self.imaddr = ((self.imaddr & 0xFF) | (d << 8)) & am
        elif o["im_we"]:
            self.imaddr = (self.imaddr + 1) & am
        commit = we and a == 0x13
        if commit and self.run:
            self.imem_err = 1
        elif we and a == 0x05 and d >> 7 & 1:
            self.imem_err = 0
        w1c = d if (we and a == 0x0E) else 0
        new = 0
        for bit, src in ((0, "ill_err"), (1, "stk_err"), (3, "tx_underrun"), (4, "ctrl_irq")):
            old = (self.st2 >> bit) & 1
            nb = i[src] | (old & ~((w1c >> bit) & 1) & 1)
            new |= nb << bit
        self.st2 = new
        if we and a == 0x28:
            self.dm_addr = d
        elif o["dm_acc"]:
            self.dm_addr = (self.dm_addr + 1) & 0xFF


async def init(dut):
    for s in INS:
        getattr(dut, s).value = 0
    await start(dut)
    return Regs()


async def cycle(dut, m, i):
    for k, x in i.items():
        getattr(dut, k).value = x
    await settle()
    o = m.comb(i)
    for k in OUTS:
        g = v(getattr(dut, k))
        assert g == o[k], f"addr={i['r_addr']:#x} we={i['r_we']} re={i['r_re']}: {k} = {g:#x}, expected {o[k]:#x}"
    m.step(i, o)
    await fall(dut)


def base(**kw):
    i = {k: 0 for k in INS}
    i.update(txf_empty=1, rxf_empty=1)
    i.update(kw)
    return i


def rand_in():
    i = base()
    acc = random.random()
    i["r_addr"] = random.choice(MAPPED) if random.random() < 0.9 else random.getrandbits(8)
    i["r_we"] = int(acc < 0.35)
    i["r_re"] = int(0.35 <= acc < 0.6)
    i["r_wdata"] = random.getrandbits(8)
    for k in ("txf_full", "txf_empty", "txf_overrun", "rxf_full", "rxf_empty", "rxf_overrun",
              "owner_err", "rx_overrun"):
        i[k] = random.getrandbits(1)
    for k in ("ill_err", "stk_err", "tx_underrun", "ctrl_irq"):
        i[k] = int(random.random() < 0.03)
    i.update(txf_count=random.getrandbits(CW), rxf_count=random.getrandbits(CW),
             rxf_rdata=random.getrandbits(8), pc=random.getrandbits(AW), cstat_in=random.getrandbits(8),
             pin_in=random.getrandbits(NP), dm_rdata=random.getrandbits(8))
    return i


@cocotb.test(timeout_time=50, timeout_unit="ms")
async def random_access(dut):
    m = await init(dut)
    for _ in range(6000):
        await cycle(dut, m, rand_in())


@cocotb.test()
async def imem_commit(dut):
    m = await init(dut)
    start_addr = min(0xFE, DEPTH - 2)
    await cycle(dut, m, base(r_we=1, r_addr=0x0A, r_wdata=start_addr & 0xFF))
    await cycle(dut, m, base(r_we=1, r_addr=0x0D, r_wdata=start_addr >> 8))
    for wd in (0x11223344, 0xA5A5F00F, 0xDEADBEEF):
        for b in range(4):
            i = base(r_we=1, r_addr=0x10 + b, r_wdata=(wd >> (8 * b)) & 0xFF)
            if b == 3:
                assert m.comb(i)["im_wdata"] == wd and m.comb(i)["im_we"] == 1
            await cycle(dut, m, i)
    await cycle(dut, m, base(r_we=1, r_addr=0x02, r_wdata=1))          # RUN
    i = base(r_we=1, r_addr=0x13, r_wdata=0)
    assert m.comb(i)["im_we"] == 0
    await cycle(dut, m, i)
    assert m.imem_err == 1
    await cycle(dut, m, base(r_we=1, r_addr=0x05, r_wdata=0x80))
    assert m.imem_err == 0
    await cycle(dut, m, base())


@cocotb.test()
async def status2_w1c(dut):
    m = await init(dut)
    await cycle(dut, m, base(ill_err=1, stk_err=1, tx_underrun=1, ctrl_irq=1))
    await cycle(dut, m, base(r_re=1, r_addr=0x0E))
    assert m.st2 == 0b11011
    await cycle(dut, m, base(r_we=1, r_addr=0x0E, r_wdata=0b00011))     # clear ill, stk
    assert m.st2 == 0b11000
    await cycle(dut, m, base(r_we=1, r_addr=0x0E, r_wdata=0b11000, ctrl_irq=1))   # set wins
    assert m.st2 == 0b10000
    await cycle(dut, m, base(r_we=1, r_addr=0x0E, r_wdata=0b10100, rx_overrun=1))
    await cycle(dut, m, base(r_re=1, r_addr=0x0E))


@cocotb.test()
async def irq_sources(dut):
    m = await init(dut)
    for en, src in ((1, dict(rxf_empty=0)), (2, dict(txf_full=0)), (4, dict(owner_err=1)),
                    (4, dict(rx_overrun=1)), (4, dict(txf_overrun=1)), (8, dict())):
        await cycle(dut, m, base(r_we=1, r_addr=0x06, r_wdata=en))
        if en == 8:
            await cycle(dut, m, base(ctrl_irq=1))
        i = base(**{"txf_full": 1, **src})
        assert m.comb(i)["irq"] == 1, (en, src)
        await cycle(dut, m, i)
        await cycle(dut, m, base(r_we=1, r_addr=0x06, r_wdata=0))
        assert m.comb(base(**{"txf_full": 1, **src}))["irq"] == 0
        await cycle(dut, m, base(r_we=1, r_addr=0x0E, r_wdata=0x1F))


@cocotb.test()
async def dmem_auto_inc(dut):
    m = await init(dut)
    await cycle(dut, m, base(r_we=1, r_addr=0x28, r_wdata=0xFD))
    for k in range(5):
        await cycle(dut, m, base(r_we=k & 1, r_re=1 - (k & 1), r_addr=0x29, r_wdata=k))
    assert m.dm_addr == 0x02
    await cycle(dut, m, base(r_re=1, r_addr=0x28))
