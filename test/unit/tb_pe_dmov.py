# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_dmov: unit test for src/mem/pe_dmov.v (the DMEM port and the two data movers).

What is checked
---------------
1. random_vs_model: 6000 cycles of random inputs (host accesses, controller
   ld / st, mover config and GO, FIFO flags, host FIFO accesses, DMEM read
   data). Every cycle every output is compared with the Python model
   (pe_model.blocks.Dmov), and after each edge every register too.
2. tx_mover_end_to_end: a DMEM shadow (with the registered read) and a TX
   FIFO (depth 8, random host writes and random pops by "the line") around
   the block: TXM delivers dmem[ptr..ptr+n-1] to the FIFO in order, none
   lost, none twice, for n = 1, 2, 7, 8, 9, 100, 256, with the pointer
   wrapping past 0xFF. busy ends only after the last push.
3. rx_mover_end_to_end: an RX FIFO filled at random by "the line" with
   random host pops: RXM writes every byte it pops to dmem[ptr..] in order;
   count = bytes stored; it stops at n; GO 0 stops it at once.
4. arbitration: host > RXM > TXM > controller, every combination; the
   controller is granted only when no one else uses the port.
"""

import random
import sys
from pathlib import Path

import cocotb
from tbutil import fall, settle, start, v

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "model"))
from pe_model.blocks import Dmov  # noqa: E402

INS = ("restart", "h_acc", "h_we", "h_addr", "h_wdata", "c_re", "c_we", "c_addr", "c_wdata", "cfg_we", "cfg_unit",
       "cfg_idx", "cfg_data", "rd_unit", "rd_idx", "h_txf_wr", "txf_full", "h_rxf_rd", "rxf_empty", "rxf_rdata",
       "m_rdata")
OUTS = ("c_gnt", "m_re", "m_we", "m_addr", "m_wdata", "m_rxf_rd", "m_txf_wr", "txm_busy", "rxm_busy")
REGS = ("t_ptr", "t_left", "t_pend", "t_bufv", "r_ptr", "r_left", "r_cnt")


async def init(dut):
    for s in INS:
        getattr(dut, s).value = 0
    dut.txf_full.value = 0
    dut.rxf_empty.value = 1
    await start(dut)
    return Dmov()


def drive(dut, x):
    for k, val in x.items():
        getattr(dut, k).value = val


def model_outs(m, x):
    d = m.comb(x["h_acc"], x["h_we"], x["h_addr"], x["h_wdata"], x["c_re"], x["c_we"], x["c_addr"], x["c_wdata"],
               x["h_txf_wr"], x["txf_full"], x["h_rxf_rd"], x["rxf_empty"], x["rxf_rdata"])
    return d


async def step(dut, m, x, check=True):
    """Drive x, compare outputs with the model, clock both, compare registers."""
    full = {k: 0 for k in INS}
    full.update(x)
    drive(dut, full)
    await settle()
    d = model_outs(m, full)
    if check:
        exp = dict(c_gnt=d["c_gnt"], m_re=d["m_re"], m_we=d["m_we"], m_rxf_rd=d["m_rxf_rd"], m_txf_wr=d["m_txf_wr"],
                   txm_busy=m.txm_busy, rxm_busy=m.rxm_busy)
        if d["m_re"] or d["m_we"]:
            exp["m_addr"] = d["m_addr"]
        if d["m_we"]:
            exp["m_wdata"] = d["m_wdata"]
        if d["m_txf_wr"]:
            exp["m_txf_wdata"] = d["m_txf_wdata"]
        exp["rd_data"] = m.read(full["rd_unit"], full["rd_idx"])
        for k, e in exp.items():
            assert v(getattr(dut, k)) == int(e), f"{k}: RTL {v(getattr(dut, k))} model {int(e)} inputs {full}"
    await fall(dut)
    m.clock(0, full["restart"], full["cfg_we"], full["cfg_unit"], full["cfg_idx"], full["cfg_data"], d,
            full["m_rdata"])
    if check:
        for r in REGS:
            assert v(getattr(dut, r)) == int(getattr(m, r)), f"{r}: RTL {v(getattr(dut, r))} model {getattr(m, r)}"
        if m.t_bufv:
            assert v(dut.t_buf) == m.t_buf
    return d


def rand_inputs(rng):
    x = dict(h_acc=int(rng.random() < 0.15), h_we=rng.getrandbits(1), h_addr=rng.getrandbits(8),
             h_wdata=rng.getrandbits(8), c_re=int(rng.random() < 0.3), c_we=0, c_addr=rng.getrandbits(8),
             c_wdata=rng.getrandbits(8), h_txf_wr=int(rng.random() < 0.1), txf_full=int(rng.random() < 0.3),
             h_rxf_rd=int(rng.random() < 0.1), rxf_empty=int(rng.random() < 0.4), rxf_rdata=rng.getrandbits(8),
             m_rdata=rng.getrandbits(8), rd_unit=rng.choice((5, 6, 0, 7)), rd_idx=rng.getrandbits(3),
             restart=int(rng.random() < 0.005))
    if not x["c_re"]:
        x["c_we"] = int(rng.random() < 0.3)
    if rng.random() < 0.08:
        x.update(cfg_we=1, cfg_unit=rng.choice((5, 6, 5, 6, 0, 4)), cfg_idx=rng.choice((0, 1, 1, 2)),
                 cfg_data=rng.choice((0, 1, 2, 3, 8, 255, 256, 0x1FF, rng.getrandbits(16))))
    return x


@cocotb.test(timeout_time=10, timeout_unit="ms")
async def random_vs_model(dut):
    m = await init(dut)
    rng = random.Random(7)
    for _ in range(6000):
        await step(dut, m, rand_inputs(rng))


class Env:
    """DMEM (registered read) and the two FIFOs around the block, as the chip has them."""

    def __init__(self):
        self.mem = [random.getrandbits(8) for _ in range(256)]
        self.dout = 0
        self.txf, self.rxf = [], []


async def run_env(dut, m, env, cycles, line_pop=0.3, line_push=0.0, host_txw=0.0, host_rxr=0.0, rng=None,
                  until=None):
    """Run with the environment closing the loops; returns the bytes popped by the line from TXF and the
    bytes the host popped from RXF."""
    rng = rng or random.Random(1)
    line_out, host_got = [], []
    for _ in range(cycles):
        if until and until(m):
            break
        h_txf_wr = int(rng.random() < host_txw and len(env.txf) < 8)
        h_rxf_rd = int(rng.random() < host_rxr and len(env.rxf) > 0)
        x = dict(txf_full=int(len(env.txf) >= 8), h_txf_wr=h_txf_wr, rxf_empty=int(not env.rxf),
                 rxf_rdata=env.rxf[0] if env.rxf else 0, h_rxf_rd=h_rxf_rd, m_rdata=env.dout)
        d = await step(dut, m, x)
        # DMEM
        if d["m_re"]:
            env.dout = env.mem[d["m_addr"]]
        if d["m_we"]:
            env.mem[d["m_addr"]] = d["m_wdata"]
        # TXF: host first, then the mover; the line pops
        if h_txf_wr:
            env.txf.append(("host",))
        elif d["m_txf_wr"]:
            env.txf.append(d["m_txf_wdata"])
        if env.txf and rng.random() < line_pop:
            b = env.txf.pop(0)
            if b != ("host",):
                line_out.append(b)
        # RXF: host or mover pops; the line pushes
        if h_rxf_rd:
            host_got.append(env.rxf.pop(0))
        elif d["m_rxf_rd"]:
            env.rxf.pop(0)
        if len(env.rxf) < 8 and rng.random() < line_push:
            env.rxf.append(rng.getrandbits(8))
    return line_out, host_got


@cocotb.test(timeout_time=40, timeout_unit="ms")
async def tx_mover_end_to_end(dut):
    m = await init(dut)
    env = Env()
    for n, ptr in ((1, 0), (2, 0x10), (7, 0x40), (8, 0), (9, 0xFC), (100, 0x80), (256, 0x33)):
        want = [env.mem[(ptr + i) & 0xFF] for i in range(n)]
        await step(dut, m, dict(cfg_we=1, cfg_unit=5, cfg_idx=0, cfg_data=ptr, m_rdata=env.dout))
        await step(dut, m, dict(cfg_we=1, cfg_unit=5, cfg_idx=1, cfg_data=n, m_rdata=env.dout))
        assert v(dut.txm_busy)
        out, _ = await run_env(dut, m, env, 40 * n + 200, line_pop=0.25, host_txw=0.02,
                               until=lambda mm: not mm.txm_busy)
        out2, _ = await run_env(dut, m, env, 200, line_pop=1.0, until=lambda mm: False)  # drain the FIFO
        got = out + out2
        assert got == want, f"n={n} ptr={ptr:#x}: TXM delivered {len(got)} bytes, wrong or out of order"
        assert not v(dut.txm_busy)


@cocotb.test(timeout_time=40, timeout_unit="ms")
async def rx_mover_end_to_end(dut):
    m = await init(dut)
    env = Env()
    rng = random.Random(3)
    for n, ptr in ((1, 0x80), (5, 0xFE), (64, 0x81), (256, 0x00)):
        await step(dut, m, dict(cfg_we=1, cfg_unit=6, cfg_idx=0, cfg_data=ptr, rxf_empty=1))
        await step(dut, m, dict(cfg_we=1, cfg_unit=6, cfg_idx=1, cfg_data=n, rxf_empty=1))
        pushed = []
        stored_before = list(env.mem)
        # the line pushes; remember what it pushed in order
        k = 0
        while m.rxm_busy and k < 60 * n + 500:
            k += 1
            stored_n = sum(1 for b in pushed if b is not None)
            push = len(env.rxf) < 8 and rng.random() < 0.3 and stored_n + len(env.rxf) < n + 3
            h_rd = int(rng.random() < 0.03 and len(env.rxf) > 0)
            x = dict(rxf_empty=int(not env.rxf), rxf_rdata=env.rxf[0] if env.rxf else 0, h_rxf_rd=h_rd,
                     m_rdata=env.dout)
            d = await step(dut, m, x)
            if d["m_we"]:
                env.mem[d["m_addr"]] = d["m_wdata"]
            if h_rd:
                env.rxf.pop(0)
                pushed.append(None)                        # the host took one: not stored
            elif d["m_rxf_rd"]:
                pushed.append(env.rxf.pop(0))
            if push:
                env.rxf.append(rng.getrandbits(8))
        stored = [b for b in pushed if b is not None]
        assert not m.rxm_busy, f"n={n}: RXM never finished"
        assert v(dut.r_cnt) == n == len(stored)
        assert [env.mem[(ptr + i) & 0xFF] for i in range(n)] == stored, f"n={n}: wrong bytes in DMEM"
        others = [i for i in range(256) if (i - ptr) & 0xFF >= n]
        assert all(env.mem[i] == stored_before[i] for i in others), "RXM wrote outside its buffer"
        env.rxf.clear()
    # GO 0 stops RXM at once, keeping the count
    await step(dut, m, dict(cfg_we=1, cfg_unit=6, cfg_idx=1, cfg_data=10, rxf_empty=1))
    await step(dut, m, dict(rxf_empty=0, rxf_rdata=0x42))
    await step(dut, m, dict(cfg_we=1, cfg_unit=6, cfg_idx=1, cfg_data=0, rxf_empty=0))
    assert not v(dut.rxm_busy) and v(dut.r_cnt) == 0
    await step(dut, m, dict(rxf_empty=0, rxf_rdata=0x43))
    assert not v(dut.m_rxf_rd)


@cocotb.test(timeout_time=10, timeout_unit="ms")
async def arbitration(dut):
    m = await init(dut)
    # keep both movers busy: TXM has bytes to read, RXM has a non-empty FIFO
    for h_acc in (0, 1):
        for r_on in (0, 1):
            for t_on in (0, 1):
                for c_re, c_we in ((0, 0), (1, 0), (0, 1)):
                    await step(dut, m, dict(restart=1))
                    if t_on:
                        await step(dut, m, dict(cfg_we=1, cfg_unit=5, cfg_idx=1, cfg_data=4))
                    if r_on:
                        await step(dut, m, dict(cfg_we=1, cfg_unit=6, cfg_idx=1, cfg_data=4))
                    x = dict(h_acc=h_acc, h_we=1, h_addr=0x11, c_re=c_re, c_we=c_we, c_addr=0x22,
                             rxf_empty=0, rxf_rdata=0x5A)
                    drive(dut, {**{k: 0 for k in INS}, **x})
                    await settle()
                    users = [u for u, on in (("host", h_acc), ("rxm", r_on), ("txm", t_on),
                                             ("ctrl", c_re or c_we)) if on]
                    owner = users[0] if users else None
                    gnt = v(dut.c_gnt)
                    assert gnt == int(not (h_acc or r_on or t_on)), (h_acc, r_on, t_on, gnt)
                    acc = v(dut.m_re) + v(dut.m_we)
                    assert acc == (1 if owner else 0), (owner, acc)
                    if owner == "host":
                        assert v(dut.m_we) and v(dut.m_addr) == 0x11
                    elif owner == "rxm":
                        assert v(dut.m_we) and v(dut.m_wdata) == 0x5A and v(dut.m_rxf_rd)
                    elif owner == "txm":
                        assert v(dut.m_re) and not v(dut.m_rxf_rd)
                    elif owner == "ctrl":
                        assert v(dut.m_addr) == 0x22 and (v(dut.m_re) == c_re) and (v(dut.m_we) == c_we)
                    await fall(dut)
                    m = Dmov()
                    await step(dut, m, dict(restart=1), check=False)
