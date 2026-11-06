# SPDX-License-Identifier: Apache-2.0
"""
blocks: cycle-accurate models of the leaf and host blocks.

What this file does
-------------------
One class per RTL module: pe_fifo, pe_imem, pe_dmem, pe_pins + pe_sync, pe_vote,
pe_crc, pe_host_if, pe_regs. Each holds that module's registers and has:
  * properties or methods for its combinational outputs, and
  * clock(...): the next state at a rising edge, computed only from the current
    state and the values passed in, so the chip model can compute every
    combinational signal first and then clock all blocks.
`None` means "unknown" (the RTL would hold X): unwritten memory words. Reset is
synchronous and active high, as in the RTL.
"""

from .config import Config


def bits(v, w):
    return v & ((1 << w) - 1)


def mux(c, a, b):
    """a if c else b, without a branch when the proof in model/equiv/ runs the model symbolically."""
    return c.mux(a, b) if hasattr(c, "mux") else (a if c else b)


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
        # read-before-write; past the end an address reads 0 (nop) and a write does nothing
        self.rdata = self.mem[raddr] if raddr < len(self.mem) else 0
        if we and waddr < len(self.mem):
            self.mem[waddr] = bits(wdata, self.iw)


# --------------------------------------------------------------- DMEM
class Dmem:
    """Combinational read, one write per cycle, no reset. The host has priority."""

    def __init__(self, depth):
        self.depth = depth
        self.mem = [None] * depth

    def rdata(self, addr):
        return self.mem[addr % self.depth]

    def clock(self, we, addr, wdata):
        if we:
            self.mem[addr % self.depth] = wdata & 0xFF


# --------------------------------------------------------------- pins
class Pins:
    """pe_pins and its pe_sync (docs/spec/blocks.md Section 7)."""

    def __init__(self, cfg: Config):
        self.n = cfg.num_pins
        self.m = (1 << cfg.num_pins) - 1
        self.out_r, self.oe_r, self.owner_err = self.m, 0, 0
        self.s1 = self.s2 = 0
        self.filt = self.f_out = 0
        self.f_cnt = [0] * cfg.num_pins

    @property
    def pin_in(self):
        return (self.f_out & self.filt) | (self.s2 & ~self.filt)

    def pad_out(self, od):
        return self.out_r & ~od & self.m

    def pad_oe(self, od):
        return ((od & ~self.out_r) | (~od & self.oe_r)) & self.m

    def clock(self, rst, c, t, r, tx_busy, rx_busy, owner, clr, pad_in, filt=0):
        """c, t, r: (wm, wv, om, ov) of the CTRL, TXP and RXP ports.
        filt: PIN_FILT. A filtered input takes a new level only after 4 samples in a row have it, so
        it rejects anything shorter than 3 clock periods: 62 ns at 48 MHz, covering the 50 ns
        spikes of I2C (UM10204 Table 10, tSP). It adds 4 cycles of input delay on that pin."""
        if rst:
            self.out_r, self.oe_r, self.owner_err = self.m, 0, 0
            self.s1 = self.s2 = 0
            self.filt = self.f_out = 0
            self.f_cnt = [0] * self.n
            return
        own_t = own_r = ok_c = 0
        for i in range(self.n):
            o = (owner >> (2 * i)) & 3
            b = 1 << i
            is_t, is_r = o == 1, o == 2
            own_t |= is_t * b
            own_r |= is_r * b
            ok_c |= b * (is_t * (1 - tx_busy) + is_r * (1 - rx_busy) + (1 - is_t - is_r))
        t_wh, t_oh = t[0] & own_t, t[2] & own_t
        r_wh = r[0] & own_r
        c_wh = c[0] & ok_c & ~t_wh & ~r_wh
        c_oh = c[2] & ok_c & ~t_oh
        wr_hit = t_wh | r_wh | c_wh
        wr_val = (t[1] & t_wh) | (r[1] & r_wh) | (c[1] & c_wh)
        oe_hit = t_oh | c_oh
        oe_val = (t[3] & t_oh) | (c[3] & c_oh)
        denied = (t[0] & ~own_t) or (t[2] & ~own_t) or (r[0] & ~own_r) or (c[0] & ~ok_c) or (c[2] & ~ok_c)
        self.out_r = ((self.out_r & ~wr_hit) | wr_val) & self.m
        self.oe_r = ((self.oe_r & ~oe_hit) | oe_val) & self.m
        if denied:
            self.owner_err = 1
        elif clr:
            self.owner_err = 0
        for i in range(self.n):
            b = 1 << i
            if not (self.filt & b):
                self.f_out = (self.f_out & ~b) | (self.s2 & b)
                self.f_cnt[i] = 0
            elif (self.s2 ^ self.f_out) & b:
                if self.f_cnt[i] == 3:
                    self.f_out ^= b
                    self.f_cnt[i] = 0
                else:
                    self.f_cnt[i] += 1
            else:
                self.f_cnt[i] = 0
        self.filt = filt & self.m
        self.s1, self.s2 = pad_in & self.m, self.s1


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
        if start & (self.running ^ 1):
            self.running = 1
            self.sel, self.n_l, self.k_l, self.s_l = pin_sel, n_cfg, k_cfg, s_cfg
            self.scnt = mux(s_cfg != 0, s_cfg - 1, 0)
            self.idx = self.ones = 0
        elif self.running:
            if self.scnt == 0:
                sample = (pin_in >> self.sel) & 1
                in_win = (self.idx >= self.k_l) & (self.idx + self.k_l < self.n_l)
                ones_nx = bits(self.ones + (in_win & sample), self.nw)
                self.ones = ones_nx
                self.scnt = mux(self.s_l != 0, self.s_l - 1, 0)
                if self.idx + 1 >= self.n_l:
                    self.running = 0
                    done = 1
                    votes = self.n_l - 2 * self.k_l
                    self.bit = 2 * ones_nx > mux(votes > 0, votes, 0)
                else:
                    self.idx = bits(self.idx + 1, self.nw)
            else:
                self.scnt -= 1
        self.done = done


# ---------------------------------------------------------------- CRC
class Crc:
    """pe_crc (docs/spec/blocks.md Section 5a)."""

    def __init__(self):
        self.src = self.app = self.c_skip = self.poly = 0
        self.crc = self.skip = self.sent = 0

    @property
    def app_ok(self):
        return int(self.app and not self.sent)

    @property
    def value(self):
        """The word TX sends: ~CRC."""
        return ~self.crc & 0xFFFF

    def read(self, idx):
        return self.crc if idx == 0 else (self.sent if idx == 1 else 0)

    def clock(self, rst, cfg_we, cfg_idx, cfg_data, tx_v, tx_bit, rx_v, rx_bit, sent_set):
        if rst:
            self.__init__()
            return
        if sent_set:
            self.sent = 1
        v = (self.src == 1 and tx_v) or (self.src == 2 and rx_v)
        b = tx_bit if self.src == 1 else rx_bit
        if v:
            if self.skip:
                self.skip -= 1
            else:
                self.crc = ((self.crc << 1) ^ (self.poly if ((self.crc >> 15) ^ b) & 1 else 0)) & 0xFFFF
        if cfg_we:
            if cfg_idx == 0:
                self.src, self.app, self.c_skip = cfg_data & 3, (cfg_data >> 2) & 1, cfg_data >> 8
            elif cfg_idx == 1:
                self.poly = cfg_data
            elif cfg_idx == 2:
                self.crc, self.skip, self.sent = cfg_data, self.c_skip, 0


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
A_TXF, A_RXF, A_IMADDR, A_PIN_IN, A_PIN_IN_H, A_IMADDR_H, A_STATUS2, A_DOORBELL = range(8, 16)
A_IMDATA, A_PIN_OD, A_OWNER, A_START_PC, A_PC, A_CSTAT = 0x10, 0x14, 0x18, 0x20, 0x22, 0x24
A_DMADDR, A_DMDATA = 0x28, 0x29
A_PIN_FILT = 0x16        # 2 bytes, as PIN_OD: input filter per pin (Pins.clock)


class Regs:
    """pe_regs (docs/spec/blocks.md Section 9)."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.run = self.start_pc = self.owner = self.od = self.filt = 0
        self.imaddr = self.stage = self.irq_en = self.imem_err = 0
        self.st2 = self.dm_addr = 0

    def comb(self, h):
        """h: the host interface's combinational outputs."""
        a, wd, we, re = h["r_addr"], h["r_wdata"], h["r_we"], h["r_re"]
        errclr = we & (a == A_ERR_CLR)
        commit = we & (a == A_IMDATA + 3)
        return dict(
            im_commit=commit, im_we=commit & (self.run ^ 1), im_waddr=self.imaddr,
            im_wdata=(wd << 24) | self.stage,
            txf_wr=we & (a == A_TXF), txf_wdata=wd,
            rxf_rd=re & (a == A_RXF),
            txf_clr=errclr & (wd >> 4), rxf_clr=errclr & (wd >> 5), owner_clr=errclr & (wd >> 6),
            restart=we & (a == A_RESTART) & wd,
            db_set=we & (a == A_DOORBELL),
            rx_ovr_clr=we & (a == A_STATUS2) & (wd >> 2),
            dm_acc=(we | re) & (a == A_DMDATA), dm_we=we & (a == A_DMDATA), dm_addr=self.dm_addr, dm_wdata=wd,
        )

    def status2(self, rx_overrun):
        return (self.st2 & 0x03) | (rx_overrun << 2) | (self.st2 & 0x18)

    def irq(self, st):
        any_err = (st["txf_overrun"] | st["rxf_overrun"] | st["owner_err"] | self.imem_err
                   | ((self.status2(st["rx_overrun"]) & 0xF) != 0))
        en = self.irq_en
        return ((en & (st["rxf_empty"] ^ 1)) | ((en >> 1) & (st["txf_full"] ^ 1)) | ((en >> 2) & any_err)
                | ((en >> 3) & (self.st2 >> 4))) & 1

    def read(self, a, st):
        if a == A_ID:
            return self.cfg.chip_id
        if a == A_VERSION:
            return self.cfg.version
        if a == A_RUN:
            return self.run
        if a == A_STATUS:
            return (st["txf_empty"] | st["txf_full"] << 1 | st["rxf_empty"] << 2 | st["rxf_full"] << 3
                    | st["txf_overrun"] << 4 | st["rxf_overrun"] << 5 | st["owner_err"] << 6
                    | self.imem_err << 7)
        if a == A_IRQ_EN:
            return self.irq_en
        if a == A_FIFO_CNT:
            return (st["txf_count"] & 0xF) | (st["rxf_count"] & 0xF) << 4
        if a == A_RXF:
            return st["rxf_rdata"]
        if a == A_IMADDR:
            return self.imaddr & 0xFF
        if a == A_IMADDR_H:
            return self.imaddr >> 8
        if a == A_PIN_IN:
            return st["pin_in"] & 0xFF
        if a == A_PIN_IN_H:
            return st["pin_in"] >> 8
        if a == A_STATUS2:
            return self.status2(st["rx_overrun"])
        if A_PIN_OD <= a <= A_PIN_OD + 1:
            return (self.od >> (8 * (a - A_PIN_OD))) & 0xFF
        if A_PIN_FILT <= a <= A_PIN_FILT + 1:
            return (self.filt >> (8 * (a - A_PIN_FILT))) & 0xFF
        if A_OWNER <= a <= A_OWNER + 3:
            return (self.owner >> (8 * (a - A_OWNER))) & 0xFF
        if A_START_PC <= a <= A_START_PC + 1:
            return (self.start_pc >> (8 * (a - A_START_PC))) & 0xFF
        if A_PC <= a <= A_PC + 1:
            return (st["pc"] >> (8 * (a - A_PC))) & 0xFF
        if a == A_CSTAT:
            return (self.run | st["blocked"] << 1 | st["z"] << 2 | st["c"] << 3 | st["to"] << 4
                    | st["etx"] << 5 | st["erx"] << 6 | st["db"] << 7)
        if a == A_DMADDR:
            return self.dm_addr
        if a == A_DMDATA:
            return st["dm_rdata"]
        return 0

    def clock(self, rst, h, rc, ill, stk, underrun, ctrl_irq):
        if rst:
            self.reset()
            return
        c = self.cfg
        a, wd, we = h["r_addr"], h["r_wdata"], h["r_we"]

        def wr(addr):
            return we & (a == addr)

        def byte_in(old, k, w):
            sh = 8 * k
            return bits((old & ~(0xFF << sh)) | (wd << sh), w)

        run = self.run
        self.run = mux(wr(A_RUN), wd & 1, run)
        self.irq_en = mux(wr(A_IRQ_EN), wd & 0xF, self.irq_en)
        for k in range(3):
            self.stage = mux(wr(A_IMDATA + k), byte_in(self.stage, k, 24), self.stage)
        for k in range(2):
            self.od = mux(wr(A_PIN_OD + k), byte_in(self.od, k, c.num_pins), self.od)
            self.filt = mux(wr(A_PIN_FILT + k), byte_in(self.filt, k, c.num_pins), self.filt)
            self.start_pc = mux(wr(A_START_PC + k), byte_in(self.start_pc, k, c.aw), self.start_pc)
        for k in range(4):
            self.owner = mux(wr(A_OWNER + k), byte_in(self.owner, k, 2 * c.num_pins), self.owner)
        self.imaddr = mux(wr(A_IMADDR), bits((self.imaddr & ~0xFF) | wd, c.aw),
                          mux(wr(A_IMADDR_H), bits((self.imaddr & 0xFF) | (wd << 8), c.aw),
                              mux(rc["im_we"], bits(self.imaddr + 1, c.aw), self.imaddr)))
        self.imem_err = mux(rc["im_commit"] & run, 1, mux(wr(A_ERR_CLR) & (wd >> 7), 0, self.imem_err))
        wc = mux(wr(A_STATUS2), wd, 0)
        st2 = self.st2
        self.st2 = ((ill | (st2 & 1 & ~wc)) & 1) | ((stk << 1 | (st2 & 2 & ~wc)) & 2) \
            | ((underrun << 3 | (st2 & 8 & ~wc)) & 8) | ((ctrl_irq << 4 | (st2 & 0x10 & ~wc)) & 0x10)
        self.dm_addr = mux(wr(A_DMADDR), wd, mux(rc["dm_acc"], (self.dm_addr + 1) & 0xFF, self.dm_addr))
