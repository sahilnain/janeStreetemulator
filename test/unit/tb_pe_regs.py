# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_regs: unit test for src/host/pe_regs.v (register file).

What is checked (against the register map in the pe_regs.v header)
--------------------------------------------------------------------
The test drives the register bus directly and drives every status input.
1. reset_values: ID, VERSION, RUN = 0, vote N/K/S reset values on the
   outputs and in readback, no pin owner / open-drain set.
2. rw_registers: random write + readback of every RW register, with
   width masking (only implemented bits read back), and the matching
   output ports carry the value (packed owner map, OD, START_PC, CFG,
   VOTE_*).
3. unmapped_read_zero: reading unmapped addresses gives 0.
4. restart_pulse: RESTART gives a one-cycle pulse on exactly the written bits.
5. imem_commit: staging bytes + the last byte give one im_we with the full
   word at IMEM_ADDR, then IMEM_ADDR auto-increments. A commit while
   running is dropped and sets imem_err; ERR_CLR bit 7 clears it.
6. fifo_strobes: TXF write -> txf_wr pulse with data; RXF read ->
   rxf_rd pulse, and r_rdata = rxf_rdata.
7. status_irq: STATUS / FIFO_CNT / PIN_IN / CSTAT / PC follow their inputs,
   ERR_CLR bits give clear pulses, irq follows IRQ_EN.
"""

import random

import cocotb
from tbutil import fall, param, settle, start, v

ID, VERSION, RUN, RESTART, STATUS, ERR_CLR, IRQ_EN, FIFO_CNT = range(8)
TXF, RXF, IMEM_ADDR, PIN_IN = 0x08, 0x09, 0x0A, 0x0B
IMEM_DATA, PIN_OD, PIN_OWNER, CTX0 = 0x10, 0x14, 0x18, 0x20

INPUTS = ["r_addr", "r_wdata", "r_we", "r_re", "txf_full", "txf_empty", "txf_count",
          "txf_overrun", "rxf_rdata", "rxf_full", "rxf_empty", "rxf_count", "rxf_overrun",
          "ctx_pc", "ctx_ready", "ctx_vote_busy", "ctx_c", "ctx_to", "pin_in", "owner_err"]


class Cfg:
    def __init__(self):
        self.nc = param("NUM_CTX", 2)
        self.np = param("NUM_PINS", 8)
        self.iw = param("IW", 24)
        self.aw = (param("IMEM_DEPTH", 64) - 1).bit_length()
        self.ctx_w = max(1, (self.nc - 1).bit_length())
        self.nw, self.sw = param("VOTE_N_W", 4), param("VOTE_S_W", 12)
        self.nb = (self.iw + 7) // 8


async def setup(dut):
    for s in INPUTS:
        getattr(dut, s).value = 0
    dut.txf_empty.value = 1
    dut.rxf_empty.value = 1
    await start(dut)
    return Cfg()


async def wr(dut, a, x):
    dut.r_addr.value, dut.r_wdata.value, dut.r_we.value = a, x, 1
    await fall(dut)
    dut.r_we.value = 0


async def rd(dut, a):
    dut.r_addr.value = a
    await settle()
    return v(dut.r_rdata)


def field(vec, i, w):
    return (vec >> (i * w)) & ((1 << w) - 1)


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def reset_values(dut):
    c = await setup(dut)
    assert await rd(dut, ID) == 0x50
    assert await rd(dut, VERSION) == 0x01
    assert await rd(dut, RUN) == 0 and v(dut.ctx_run) == 0
    for k in range(c.nc):
        assert field(v(dut.vote_n), k, c.nw) == 9
        assert field(v(dut.vote_k), k, c.nw) == 1
        assert field(v(dut.vote_s), k, c.sw) == 48
        assert await rd(dut, CTX0 + 8 * k + 2) == 9
        assert await rd(dut, CTX0 + 8 * k + 4) == 48
    assert v(dut.pin_owner) == 0 and v(dut.pin_od) == 0


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def rw_registers(dut):
    c = await setup(dut)
    regs = []  # (address, implemented bit mask, check-output function)
    regs.append((RUN, (1 << c.nc) - 1, lambda x: v(dut.ctx_run) == x))
    regs.append((IRQ_EN, 0x7, None))
    regs.append((IMEM_ADDR, (1 << c.aw) - 1, None))
    for b in range(2):
        m = ((1 << c.np) - 1) >> (8 * b) & 0xFF
        regs.append((PIN_OD + b, m, (lambda b: lambda x: (v(dut.pin_od) >> (8 * b)) & 0xFF == x)(b)))
    ow = c.np * c.ctx_w
    for b in range(8):
        m = ((1 << ow) - 1) >> (8 * b) & 0xFF
        regs.append((PIN_OWNER + b, m, (lambda b: lambda x: (v(dut.pin_owner) >> (8 * b)) & 0xFF == x)(b)))
    for k in range(c.nc):
        base = CTX0 + 8 * k
        regs.append((base + 0, (1 << c.aw) - 1, (lambda k: lambda x: field(v(dut.ctx_start_pc), k, c.aw) == x)(k)))
        regs.append((base + 1, 1, (lambda k: lambda x: (v(dut.ctx_shift_msb) >> k) & 1 == x)(k)))
        regs.append((base + 2, (1 << c.nw) - 1, (lambda k: lambda x: field(v(dut.vote_n), k, c.nw) == x)(k)))
        regs.append((base + 3, (1 << c.nw) - 1, (lambda k: lambda x: field(v(dut.vote_k), k, c.nw) == x)(k)))
        regs.append((base + 4, ((1 << c.sw) - 1) & 0xFF,
                     (lambda k: lambda x: field(v(dut.vote_s), k, c.sw) & 0xFF == x)(k)))
        regs.append((base + 5, ((1 << c.sw) - 1) >> 8,
                     (lambda k: lambda x: field(v(dut.vote_s), k, c.sw) >> 8 == x)(k)))
    for _ in range(4):
        random.shuffle(regs)
        for a, m, chk in regs:
            x = random.getrandbits(8)
            await wr(dut, a, x)
            assert await rd(dut, a) == x & m, f"addr {a:#04x}"
            if chk:
                assert chk(x & m), f"output of addr {a:#04x}"
    await wr(dut, RUN, 0)


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def unmapped_read_zero(dut):
    c = await setup(dut)
    used = set(range(0x0D)) | set(range(IMEM_DATA, IMEM_DATA + 4)) | {PIN_OD, PIN_OD + 1}
    used |= set(range(PIN_OWNER, PIN_OWNER + 8)) | set(range(CTX0, CTX0 + 8 * c.nc))
    for a in range(256):
        if a not in used:
            assert await rd(dut, a) == 0, f"addr {a:#04x}"


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def restart_pulse(dut):
    c = await setup(dut)
    for _ in range(10):
        x = random.getrandbits(c.nc)
        dut.r_addr.value, dut.r_wdata.value, dut.r_we.value = RESTART, x, 1
        await settle()
        assert v(dut.ctx_restart) == x
        await fall(dut)
        dut.r_we.value = 0
        await settle()
        assert v(dut.ctx_restart) == 0


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def imem_commit(dut):
    c = await setup(dut)
    await wr(dut, IMEM_ADDR, 5)
    for i in range(4):
        w = random.getrandbits(c.iw)
        for b in range(c.nb - 1):
            await wr(dut, IMEM_DATA + b, (w >> (8 * b)) & 0xFF)
            assert v(dut.im_we) == 0
        dut.r_addr.value, dut.r_wdata.value, dut.r_we.value = IMEM_DATA + c.nb - 1, w >> (8 * (c.nb - 1)), 1
        await settle()
        assert v(dut.im_we) == 1 and v(dut.im_wdata) == w and v(dut.im_waddr) == 5 + i
        await fall(dut)
        dut.r_we.value = 0
        assert await rd(dut, IMEM_ADDR) == 6 + i
    # commit while a context runs: dropped, imem_err set, address unchanged
    await wr(dut, RUN, 1)
    dut.r_addr.value, dut.r_wdata.value, dut.r_we.value = IMEM_DATA + c.nb - 1, 0, 1
    await settle()
    assert v(dut.im_we) == 0
    await fall(dut)
    dut.r_we.value = 0
    assert await rd(dut, STATUS) & 0x80
    assert await rd(dut, IMEM_ADDR) == 9
    await wr(dut, ERR_CLR, 0x80)
    assert not (await rd(dut, STATUS) & 0x80)
    await wr(dut, RUN, 0)


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def fifo_strobes(dut):
    await setup(dut)
    for _ in range(10):
        x = random.getrandbits(8)
        dut.r_addr.value, dut.r_wdata.value, dut.r_we.value = TXF, x, 1
        await settle()
        assert v(dut.txf_wr) == 1 and v(dut.txf_wdata) == x
        await fall(dut)
        dut.r_we.value = 0
        await settle()
        assert v(dut.txf_wr) == 0
        y = random.getrandbits(8)
        dut.rxf_rdata.value = y
        dut.r_addr.value, dut.r_re.value = RXF, 1
        await settle()
        assert v(dut.rxf_rd) == 1 and v(dut.r_rdata) == y
        await fall(dut)
        dut.r_re.value = 0
        await settle()
        assert v(dut.rxf_rd) == 0


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def status_irq(dut):
    c = await setup(dut)
    for _ in range(100):
        st = {s: random.getrandbits(1) for s in ("txf_empty", "txf_full", "rxf_empty", "rxf_full",
                                                 "txf_overrun", "rxf_overrun", "owner_err")}
        for s, x in st.items():
            getattr(dut, s).value = x
        exp = (st["txf_empty"] | st["txf_full"] << 1 | st["rxf_empty"] << 2 | st["rxf_full"] << 3
               | st["txf_overrun"] << 4 | st["rxf_overrun"] << 5 | st["owner_err"] << 6)
        assert await rd(dut, STATUS) == exp
        tc, rc = random.getrandbits(3), random.getrandbits(3)
        dut.txf_count.value, dut.rxf_count.value = tc, rc
        assert await rd(dut, FIFO_CNT) == tc | rc << 4
        pins = random.getrandbits(c.np)
        dut.pin_in.value = pins
        assert await rd(dut, PIN_IN) == pins & 0xFF
        en = random.getrandbits(3)
        await wr(dut, IRQ_EN, en)
        await settle()
        anyerr = st["txf_overrun"] | st["rxf_overrun"] | st["owner_err"]
        exp_irq = (en & 1 and not st["rxf_empty"]) or (en & 2 and not st["txf_full"]) or (en & 4 and anyerr)
        assert v(dut.irq) == int(bool(exp_irq))
    # per-context PC and CSTAT
    for _ in range(20):
        pcs = [random.getrandbits(c.aw) for _ in range(c.nc)]
        dut.ctx_pc.value = sum(p << (k * c.aw) for k, p in enumerate(pcs))
        flags = {s: random.getrandbits(c.nc) for s in ("ctx_ready", "ctx_vote_busy", "ctx_c", "ctx_to")}
        for s, x in flags.items():
            getattr(dut, s).value = x
        for k in range(c.nc):
            assert await rd(dut, CTX0 + 8 * k + 6) == pcs[k]
            b = lambda s: (flags[s] >> k) & 1  # noqa: E731
            assert await rd(dut, CTX0 + 8 * k + 7) == (b("ctx_ready") | b("ctx_vote_busy") << 1
                                                      | b("ctx_c") << 2 | b("ctx_to") << 3)
    # ERR_CLR pulses
    dut.r_addr.value, dut.r_wdata.value, dut.r_we.value = ERR_CLR, 0x70, 1
    await settle()
    assert v(dut.txf_clr_overrun) == 1 and v(dut.rxf_clr_overrun) == 1 and v(dut.owner_clr_err) == 1
    await fall(dut)
    dut.r_we.value = 0
