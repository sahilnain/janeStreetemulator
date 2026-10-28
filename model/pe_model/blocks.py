# SPDX-License-Identifier: Apache-2.0
"""
blocks: cycle-accurate models of the leaf/host blocks.

What this file does
-------------------
One class per RTL module (pe_fifo, pe_imem, pe_pins, pe_vote, pe_ctx,
pe_host_if, pe_regs). Each class holds that module's registers and has:
  * properties / methods for its combinational outputs (from current state,
    plus inputs where the RTL has a combinational path), and
  * clock(...): the next state at a rising edge, computed ONLY from the
    current state and the values passed in (so the chip model can compute
    every combinational signal first and then clock all blocks).
`None` means "unknown" (the RTL would hold X): unwritten FIFO entries and
IMEM words. Reset is synchronous and active high, like the RTL.
"""

from .config import Config


def bits(v, w):
    return v & ((1 << w) - 1)


def is_zero(x, w):
    """1 if x == 0 else 0, for 0 <= x < 2**w, without branching."""
    return ((x - 1) >> w) & 1


# ------------------------------------------------------------------- FIFO
class Fifo:
    def __init__(self, depth, w):
        self.depth, self.w = depth, w
        self.mem = [None] * depth
        self.wptr = self.rptr = self.cnt = 0
        self.overrun = 0

    @property
    def empty(self):
        return int(self.cnt == 0)

    @property
    def full(self):
        return int(self.cnt == self.depth)

    @property
    def rdata(self):
        return self.mem[self.rptr]

    def clock(self, rst, wr, wdata, rd, clr):
        rd_ok = rd and not self.empty
        wr_ok = wr and (not self.full or rd_ok)
        if wr_ok:  # the RTL memory write is not gated by reset
            self.mem[self.wptr] = bits(wdata, self.w)
        if rst:
            self.wptr = self.rptr = self.cnt = 0
            self.overrun = 0
            return
        if wr_ok:
            self.wptr = 0 if self.wptr == self.depth - 1 else self.wptr + 1
        if rd_ok:
            self.rptr = 0 if self.rptr == self.depth - 1 else self.rptr + 1
        self.cnt += int(bool(wr_ok)) - int(bool(rd_ok))
        if wr and not wr_ok:
            self.overrun = 1
        elif clr:
            self.overrun = 0


# --------------------------------------------------------------- IMEM
class Imem:
    def __init__(self, depth, iw):
        self.mem = [None] * depth
        self.rdata = None
        self.iw = iw

    def clock(self, we, waddr, wdata, raddr):
        self.rdata = self.mem[raddr]  # read-before-write
        if we:
            self.mem[waddr] = bits(wdata, self.iw)


# --------------------------------------------------------------- pins
class Pins:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.m = (1 << cfg.num_pins) - 1
        self.out_r, self.oe_r, self.owner_err = self.m, 0, 0
        self.s1 = self.s2 = 0

    @property
    def pin_in(self):
        return self.s2

    def pad_out(self, od):
        return self.out_r & ~od & self.m

    def pad_oe(self, od):
        return ((od & ~self.out_r) | (~od & self.oe_r)) & self.m

    def own_mask(self, owner, ctx):
        """Bit i set if pin i belongs to `ctx`. Branch-free (no per-pin `if`),
        so the symbolic equivalence proof does not fork once per pin."""
        w = self.cfg.ctx_w
        return sum(is_zero(bits(owner >> (i * w), w) ^ ctx, w) << i for i in range(self.cfg.num_pins))

    def clock(self, rst, wr_en, wr_ctx, wr_mask, wr_val, oe_en, oe_mask, oe_val, owner, clr, pad_in):
        if rst:
            self.out_r, self.oe_r, self.owner_err = self.m, 0, 0
            self.s1 = self.s2 = 0
            return
        own = self.own_mask(owner, wr_ctx)
        wh = (wr_mask & own) if wr_en else 0
        oh = (oe_mask & own) if oe_en else 0
        denied = (wr_en and (wr_mask & ~own & self.m)) or (oe_en and (oe_mask & ~own & self.m))
        self.out_r = (self.out_r & ~wh) | (wr_val & wh)
        self.oe_r = (self.oe_r & ~oh) | (oe_val & oh)
        if denied:
            self.owner_err = 1
        elif clr:
            self.owner_err = 0
        self.s1, self.s2 = bits(pad_in, self.cfg.num_pins), self.s1


# --------------------------------------------------------------- vote
class Vote:
    def __init__(self, cfg: Config):
        self.nw, self.sw = cfg.vote_n_w, cfg.vote_s_w
        self.running = self.done = self.bit = 0
        self.sel = self.n_l = self.k_l = self.s_l = self.scnt = self.idx = self.ones = 0

    @property
    def busy(self):
        return self.running | self.done

    def clock(self, rst, start, pin_sel, pin_in, n_cfg, k_cfg, s_cfg):
        if rst:
            self.running = self.done = self.bit = 0
            self.sel = self.n_l = self.k_l = self.s_l = self.scnt = self.idx = self.ones = 0
            return
        done = 0
        if start and not self.running:
            self.running = 1
            self.sel, self.n_l, self.k_l, self.s_l = pin_sel, n_cfg, k_cfg, s_cfg
            self.scnt = s_cfg - 1 if s_cfg else 0
            self.idx = self.ones = 0
        elif self.running:
            if self.scnt == 0:
                sample = (pin_in >> self.sel) & 1
                in_win = self.idx >= self.k_l and self.idx + self.k_l < self.n_l
                ones_nx = bits(self.ones + int(in_win and sample), self.nw)
                self.ones = ones_nx
                self.scnt = self.s_l - 1 if self.s_l else 0
                if self.idx + 1 >= self.n_l:
                    self.running = 0
                    done = 1
                    votes = max(self.n_l - 2 * self.k_l, 0)
                    self.bit = int(2 * ones_nx > votes)
                else:
                    self.idx = bits(self.idx + 1, self.nw)
            else:
                self.scnt -= 1
        self.done = done


# ------------------------------------------------------------ context
STATE = ("pc", "ctr0", "ctr1", "osr", "osr_cnt", "isr", "c", "to")


class Context:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.reset()

    def reset(self):
        for k in STATE:
            setattr(self, k, 0)
        self.stall = 0

    def ready(self, run, vote_busy):
        return int(bool(run) and self.stall == 0 and not vote_busy)

    def state(self):
        return {k: getattr(self, k) for k in STATE}

    def clock(self, rst, restart, start_pc, shift_msb, we, nxt, vote_done, vote_bit):
        if rst:
            self.reset()
            return
        if restart:
            self.reset()
            self.pc = start_pc
            self.stall = 1
            return
        sr = self.cfg.sr_w
        isr_vote = bits((self.isr << 1) | vote_bit, sr) if shift_msb else (vote_bit << (sr - 1)) | (self.isr >> 1)
        if we:
            for k in STATE:
                setattr(self, k, nxt[k + "_n"])
            self.stall = nxt["stall_n"]
        elif self.stall:
            self.stall -= 1
        if vote_done:
            self.isr, self.c = isr_vote, vote_bit


# ---------------------------------------------------------- host interface
class HostIf:
    OP_ADDR, OP_WRITE, OP_READ, OP_SYNC = range(4)

    def __init__(self):
        self.reset()

    def reset(self):
        self.s1 = self.s2 = 0  # pe_sync on h_stb
        self.stb_q = self.ack = 0
        self.dout = 0
        self.addr = self.wbuf = self.wphase = self.rlow = self.rphase = 0

    def comb(self, din, op):
        ev = self.s2 ^ self.stb_q
        return dict(ev=ev, r_addr=self.addr, r_wdata=(self.wbuf << 4) | din,
                    r_we=int(ev and op == self.OP_WRITE and self.wphase),
                    r_re=int(ev and op == self.OP_READ and not self.rphase))

    def clock(self, rst, din, op, stb, r_rdata):
        if rst:
            self.reset()
            return
        ev = self.s2 ^ self.stb_q
        self.stb_q = self.s2
        self.s1, self.s2 = stb, self.s1
        if not ev:
            return
        self.ack ^= 1
        if op == self.OP_ADDR:
            self.addr = ((self.addr << 4) | din) & 0xFF
        elif op == self.OP_WRITE:
            if not self.wphase:
                self.wbuf = din
            self.wphase ^= 1
        elif op == self.OP_READ:
            if not self.rphase:
                self.dout = None if r_rdata is None else r_rdata >> 4
                self.rlow = None if r_rdata is None else r_rdata & 0xF
            else:
                self.dout = self.rlow
            self.rphase ^= 1
        else:
            self.wphase = self.rphase = 0


# ------------------------------------------------------------ registers
A_ID, A_VERSION, A_RUN, A_RESTART, A_STATUS, A_ERR_CLR, A_IRQ_EN, A_FIFO_CNT = range(8)
A_TXF, A_RXF, A_IMADDR, A_PIN_IN = 0x08, 0x09, 0x0A, 0x0B
A_IMDATA, A_PIN_OD, A_OWNER, A_CTX0 = 0x10, 0x14, 0x18, 0x20


def write_vec(vec, width, base, addr, wdata):
    """Byte-addressed packed vector at `base` (byte i at base+i), like the
    per-bit loops in pe_regs.v."""
    i = addr - base
    if 0 <= i and 8 * i < width:
        for b in range(8 * i, min(8 * i + 8, width)):
            vec = (vec & ~(1 << b)) | (((wdata >> (b - 8 * i)) & 1) << b)
    return vec


def read_vec(vec, width, base, addr):
    i = addr - base
    if 0 <= i and 8 * i < width:
        return (vec >> (8 * i)) & ((1 << min(8, width - 8 * i)) - 1)
    return None


class Regs:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.reset()

    def reset(self):
        c = self.cfg
        self.run = self.msb = 0
        self.start_pc = [0] * c.num_ctx
        self.vote_n = [bits(c.vote_n_rst, c.vote_n_w)] * c.num_ctx
        self.vote_k = [bits(c.vote_k_rst, c.vote_n_w)] * c.num_ctx
        self.vote_s = [bits(c.vote_s_rst, c.vote_s_w)] * c.num_ctx
        self.owner = self.od = 0
        self.imaddr = self.stage = self.irq_en = self.imem_err = 0

    def comb(self, h, st):
        """h: host comb (r_addr, r_wdata, r_we, r_re); st: status inputs.
        Returns the register file's combinational outputs."""
        c = self.cfg
        a, wd, we, re = h["r_addr"], h["r_wdata"], h["r_we"], h["r_re"]
        any_run = int(self.run != 0)
        commit = we and a == A_IMDATA + c.nb - 1
        last = 8 * (c.nb - 1)
        im_wdata = (self.stage & ((1 << last) - 1)) | (bits(wd, c.iw - last) << last)
        any_err = st["txf_overrun"] | st["rxf_overrun"] | st["owner_err"] | self.imem_err
        irq = int(bool((self.irq_en & 1 and not st["rxf_empty"]) or (self.irq_en & 2 and not st["txf_full"])
                       or (self.irq_en & 4 and any_err)))
        errclr = we and a == A_ERR_CLR
        return dict(
            r_rdata=self.read(a, st), irq=irq,
            im_we=int(bool(commit and not any_run)), im_commit=int(bool(commit)), im_waddr=self.imaddr,
            im_wdata=im_wdata,
            txf_wr=int(bool(we and a == A_TXF)), txf_wdata=bits(wd, c.data_w),
            rxf_rd=int(bool(re and a == A_RXF)),
            txf_clr=int(bool(errclr and wd & 0x10)), rxf_clr=int(bool(errclr and wd & 0x20)),
            owner_clr=int(bool(errclr and wd & 0x40)),
            restart=bits(wd, c.num_ctx) if (we and a == A_RESTART) else 0,
        )

    def read(self, a, st):
        c = self.cfg
        if a == A_ID:
            return c.chip_id
        if a == A_VERSION:
            return c.version
        if a == A_RUN:
            return self.run
        if a == A_STATUS:
            return (st["txf_empty"] | st["txf_full"] << 1 | st["rxf_empty"] << 2 | st["rxf_full"] << 3
                    | st["txf_overrun"] << 4 | st["rxf_overrun"] << 5 | st["owner_err"] << 6
                    | self.imem_err << 7)
        if a == A_IRQ_EN:
            return self.irq_en
        if a == A_FIFO_CNT:
            cw4 = min(c.fifo_cw, 4)
            return bits(st["txf_count"], cw4) | bits(st["rxf_count"], cw4) << 4
        if a == A_RXF:
            return st["rxf_rdata"]
        if a == A_IMADDR:
            return self.imaddr
        for vec, w, base in ((st["pin_in"], c.num_pins, A_PIN_IN), (self.od, c.num_pins, A_PIN_OD),
                             (self.owner, c.num_pins * c.ctx_w, A_OWNER)):
            r = read_vec(vec, w, base, a)
            if r is not None:
                return r
        for k in range(c.num_ctx):
            off = a - (A_CTX0 + 8 * k)
            if off == 0:
                return self.start_pc[k]
            if off == 1:
                return (self.msb >> k) & 1
            if off == 2:
                return self.vote_n[k]
            if off == 3:
                return self.vote_k[k]
            if off in (4, 5):
                return read_vec(self.vote_s[k], c.vote_s_w, A_CTX0 + 8 * k + 4, a) or 0
            if off == 6:
                return st["ctx_pc"][k]
            if off == 7:
                return (st["ready"][k] | st["vote_busy"][k] << 1 | st["c"][k] << 2 | st["to"][k] << 3
                        | ((self.run >> k) & 1) << 4)
        return 0

    def clock(self, rst, h, rc):
        """h: host comb, rc: this block's comb outputs (computed this cycle)."""
        if rst:
            self.reset()
            return
        c = self.cfg
        a, wd, we = h["r_addr"], h["r_wdata"], h["r_we"]
        any_run = self.run != 0
        if we:
            if a == A_RUN:
                self.run = bits(wd, c.num_ctx)
            if a == A_IRQ_EN:
                self.irq_en = wd & 7
            self.stage = write_vec(self.stage, c.iw, A_IMDATA, a, wd)
            self.od = write_vec(self.od, c.num_pins, A_PIN_OD, a, wd)
            self.owner = write_vec(self.owner, c.num_pins * c.ctx_w, A_OWNER, a, wd)
            for k in range(c.num_ctx):
                off = a - (A_CTX0 + 8 * k)
                if off == 0:
                    self.start_pc[k] = bits(wd, c.aw)
                elif off == 1:
                    self.msb = (self.msb & ~(1 << k)) | ((wd & 1) << k)
                elif off == 2:
                    self.vote_n = self.vote_n[:k] + [bits(wd, c.vote_n_w)] + self.vote_n[k + 1:]
                elif off == 3:
                    self.vote_k = self.vote_k[:k] + [bits(wd, c.vote_n_w)] + self.vote_k[k + 1:]
                elif off in (4, 5):
                    v = write_vec(self.vote_s[k], c.vote_s_w, A_CTX0 + 8 * k + 4, a, wd)
                    self.vote_s = self.vote_s[:k] + [v] + self.vote_s[k + 1:]
        if we and a == A_IMADDR:
            self.imaddr = bits(wd, c.aw)
        elif rc["im_we"]:
            self.imaddr = bits(self.imaddr + 1, c.aw)
        if rc["im_commit"] and any_run:
            self.imem_err = 1
        elif we and a == A_ERR_CLR and wd & 0x80:
            self.imem_err = 0
