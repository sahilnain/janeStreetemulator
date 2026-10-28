# SPDX-License-Identifier: Apache-2.0
"""
chip: cycle-accurate reference model of the whole chip (tt_um_sahil_protoemu).

What this file does
-------------------
Wires the block models exactly like src/tt_um_sahil_protoemu.v:
  host bus -> HostIf -> Regs -> {Imem, config, FIFOs, status}
  Imem -> Core -> Pins -> uio ;  Core <-> TX/RX Fifo <-> Regs
API (one call per clock cycle):
  chip.outputs()                 -> what the pins show NOW (uo_out, uio_out,
                                    uio_oe, and which uo_out bits are known)
  chip.step(ui_in, uio_in, rst_n) -> apply the inputs present at the next
                                    rising edge and advance one cycle.
step() first computes every combinational signal from the current state,
then clocks every block, so block order cannot leak post-edge values.
"""

from .blocks import Fifo, HostIf, Imem, Pins, Regs
from .config import Config
from .core import Core


class Chip:
    def __init__(self, cfg: Config = None):
        self.cfg = cfg = cfg or Config()
        self.rst_q = [1, 1]  # rst_q[0], rst_q[1]; rst = rst_q[1]
        self.host = HostIf()
        self.regs = Regs(cfg)
        self.imem = Imem(cfg.imem_depth, cfg.iw)
        self.txf = Fifo(cfg.fifo_depth, cfg.data_w)
        self.rxf = Fifo(cfg.fifo_depth, cfg.data_w)
        self.core = Core(cfg)
        self.pins = Pins(cfg)
        self.cycle = 0

    @property
    def rst(self):
        return self.rst_q[1]

    def _status(self, cc):
        ctxs = self.core.ctxs
        return dict(txf_empty=self.txf.empty, txf_full=self.txf.full, txf_count=self.txf.cnt,
                    txf_overrun=self.txf.overrun, rxf_empty=self.rxf.empty, rxf_full=self.rxf.full,
                    rxf_count=self.rxf.cnt, rxf_overrun=self.rxf.overrun, rxf_rdata=self.rxf.rdata,
                    owner_err=self.pins.owner_err, pin_in=self.pins.pin_in,
                    ctx_pc=[c.pc for c in ctxs], ready=cc["ready"], vote_busy=cc["vote_busy"],
                    c=[c.c for c in ctxs], to=[c.to for c in ctxs])

    def _core_comb(self):
        return self.core.comb(self.regs.run, self.regs.msb, self.imem.rdata, self.pins.pin_in,
                              self.txf.rdata, self.txf.empty, self.rxf.full)

    def irq(self):
        r = self.regs
        any_err = self.txf.overrun | self.rxf.overrun | self.pins.owner_err | r.imem_err
        return int(bool((r.irq_en & 1 and not self.rxf.empty) or (r.irq_en & 2 and not self.txf.full)
                        or (r.irq_en & 4 and any_err)))

    def outputs(self):
        h = self.host
        dout = h.dout
        uo = (0 if dout is None else dout) | (h.ack << 4) | (self.irq() << 5)
        known = 0xF0 if dout is None else 0xFF
        od = self.regs.od
        return dict(uo_out=uo, uo_known=known, uio_out=self.pins.pad_out(od), uio_oe=self.pins.pad_oe(od))

    def step(self, ui_in, uio_in, rst_n=1):
        rst = self.rst
        din, op, stb = ui_in & 0xF, (ui_in >> 4) & 3, (ui_in >> 6) & 1
        r = self.regs

        # ---- combinational signals for this cycle (all from pre-edge state)
        h = self.host.comb(din, op)
        cc = self._core_comb()
        rc = r.comb(h, self._status(cc))
        run, msb, owner = r.run, r.msb, r.owner
        spc, vn, vk, vs = list(r.start_pc), list(r.vote_n), list(r.vote_k), list(r.vote_s)
        pin_in = self.pins.pin_in

        # ---- clock edge
        self.host.clock(rst, din, op, stb, rc["r_rdata"])
        self.imem.clock(rc["im_we"], rc["im_waddr"], rc["im_wdata"], cc["im_raddr"])
        self.txf.clock(rst, rc["txf_wr"], rc["txf_wdata"], cc["txf_rd"], rc["txf_clr"])
        self.rxf.clock(rst, cc["rxf_wr"], cc["rxf_wdata"], rc["rxf_rd"], rc["rxf_clr"])
        self.core.clock(rst, rc["restart"], spc, msb, vn, vk, vs, cc, pin_in)
        self.pins.clock(rst, cc["pin_wr_en"], cc["cur"], cc["pin_wr_mask"], cc["pin_wr_val"],
                        cc["pin_oe_en"], cc["pin_oe_mask"], cc["pin_oe_val"], owner,
                        rc["owner_clr"], uio_in)
        r.clock(rst, h, rc)
        self.rst_q = [1 - (rst_n & 1), self.rst_q[0]]
        self.cycle += 1
        _ = run  # (run is read by the core via regs state above)
