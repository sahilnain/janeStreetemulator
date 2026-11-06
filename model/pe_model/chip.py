# SPDX-License-Identifier: Apache-2.0
"""
chip: cycle-accurate reference model of the whole chip (tt_um_sahil_protoemu).

What this file does
-------------------
Wires the block models as src/tt_um_sahil_protoemu.v does. One call per clock:
  chip.outputs()                  what the pins show now (uo_out, uio_out, uio_oe, and
                                  which uo_out bits are known)
  chip.step(ui_in, uio_in, rst_n) apply the inputs of the next rising edge and advance.
step() first computes every combinational signal from the current state, then
clocks every block, so block order cannot leak post-edge values.
"""

from .blocks import Crc, Dmem, Fifo, HostIf, Imem, Pins, Regs
from .config import Config
from .ctrl import Ctrl
from .det import Det
from .rxu import Rxu
from .txu import Txu


class Chip:
    def __init__(self, cfg: Config = None):
        self.cfg = cfg = cfg or Config()
        self.rst_q = [1, 1]
        self.host = HostIf()
        self.regs = Regs(cfg)
        self.imem = Imem(cfg.imem_depth, 32)
        self.dmem = Dmem(cfg.dmem_depth)
        self.txf = Fifo(cfg.fifo_depth, 8)
        self.rxf = Fifo(cfg.fifo_depth, 8)
        self.ctrl = Ctrl(cfg)
        self.tx = Txu(cfg)
        self.rx = Rxu(cfg)
        self.etx = Det(0, cfg)
        self.erx = Det(1, cfg)
        self.crc = Crc()
        self.pins = Pins(cfg)
        self.cycle = 0

    @property
    def rst(self):
        return self.rst_q[1]

    @property
    def rx_busy(self):
        return int(bool(self.rx.busy or self.erx.apend))

    def _reg_status(self, dm_rdata):
        c = self.ctrl
        return dict(txf_empty=self.txf.empty, txf_full=self.txf.full, txf_count=self.txf.cnt,
                    txf_overrun=self.txf.overrun, rxf_empty=self.rxf.empty, rxf_full=self.rxf.full,
                    rxf_count=self.rxf.cnt, rxf_overrun=self.rxf.overrun, rxf_rdata=self.rxf.rdata,
                    owner_err=self.pins.owner_err, rx_overrun=self.rx.overrun, pin_in=self.pins.pin_in,
                    pc=c.pc, blocked=c.blocked, z=c.z, c=c.c, to=c.to, db=c.db, etx=self.etx.flag,
                    erx=self.erx.flag, dm_rdata=dm_rdata)

    def outputs(self):
        h = self.host
        dout = h.dout
        irq = self.regs.irq(self._reg_status(None))
        uo = (0 if dout is None else dout) | (h.ack << 4) | (irq << 5)
        known = 0xF0 if dout is None else 0xFF
        od = self.regs.od
        return dict(uo_out=uo, uo_known=known, uio_out=self.pins.pad_out(od), uio_oe=self.pins.pad_oe(od))

    def _rd(self, unit, idx):
        if unit == 0:
            return self.tx.read(idx)
        if unit == 1:
            return self.rx.read(idx, self.erx.ferr)
        if unit == 2:
            return self.etx.stat if idx == 0 else 0
        if unit == 3:
            return self.erx.stat if idx == 0 else 0
        if unit == 4:
            return self.crc.read(idx)
        return 0

    def step(self, ui_in, uio_in, rst_n=1):
        rst = self.rst
        din, op, stb = ui_in & 0xF, (ui_in >> 4) & 3, (ui_in >> 6) & 1
        tx, rx, etx, erx, crc = self.tx, self.rx, self.etx, self.erx, self.crc
        txf, rxf = self.txf, self.rxf
        pin_in = self.pins.pin_in
        tx_busy, rx_busy = tx.busy, self.rx_busy

        # host side
        h = self.host.comb(din, op)
        rc = self.regs.comb(h)
        restart = rc["restart"]

        # detectors and the TX chain decision do not depend on the controller
        te = etx.trigger(pin_in)
        tr = erx.trigger(pin_in)
        next_crc = int(txf.empty and crc.app_ok)
        next_ok = int(not txf.empty or crc.app_ok)
        next_data = crc.value if txf.empty else (txf.rdata or 0)
        probe = tx.comb(0, 0, 0, next_ok, next_crc, next_data, pin_in)
        chain_raw = probe["chain"] and not next_crc
        etx_raw = te["start_req"] and not tx_busy
        erx_raw = tr["start_req"] and not rx_busy
        det_pop = (((etx_raw and tx.den) or (erx_raw and rx.need)) and not txf.empty) or chain_raw
        rx_push_early = rx.comb(0, 0, 0, pin_in, probe["stick"], probe["dtick"])["push"]

        dm_rd = self.dmem.rdata(rc["dm_addr"]) if rc["dm_acc"] else None
        s = dict(pin_in=pin_in, tx_busy=tx_busy, rx_busy=rx_busy, tx_need=tx.den, rx_need=rx.need,
                 etx=etx.flag, erx=erx.flag, ferr=erx.ferr, rxrdy=rx.rxrdy, txf_empty=txf.empty, txf_full=txf.full,
                 txf_rdata=txf.rdata, rxf_empty=rxf.empty, rxf_full=rxf.full, rxf_rdata=rxf.rdata,
                 txf_rd_gnt=not det_pop, txf_wr_gnt=not rc["txf_wr"], rxf_rd_gnt=not rc["rxf_rd"],
                 rxf_wr_gnt=not rx_push_early, dm_gnt=not rc["dm_acc"])
        e = self.ctrl.comb(self.regs.run, self.imem.rdata, s, self._rd, self.dmem.rdata)

        # starts, as in the top level
        sm, stm = e["start_mask"], e["stop_mask"]
        use_imm = sm and e["start_imm"]
        byte_avail = use_imm or not txf.empty
        etx_try = etx_raw and not stm & 1 and not restart
        erx_try = erx_raw and not stm & 2 and not restart
        tx_go = int(bool((sm & 1 and not restart) or (etx_try and (not tx.den or byte_avail))))
        rx_go = int(bool((sm & 2 and not restart) or (erx_try and (not rx.need or byte_avail))))
        underrun = int(bool((etx_try and tx.den and not byte_avail) or (erx_try and rx.need and not byte_avail)))
        u_pop = ((tx_go and tx.den) or (rx_go and rx.need)) and not use_imm
        u_data = e["start_data"] if use_imm else (0 if txf.empty else txf.rdata)
        etx_stop, erx_stop = te["stop_req"], tr["stop_req"]
        tx_stop = int(bool(stm & 1 or etx_stop or restart))
        rx_stop = int(bool(stm & 2 or erx_stop or restart))

        t = tx.comb(tx_go, u_data, tx_stop, next_ok, next_crc, next_data, pin_in)
        chain_pop = t["chain"] and not next_crc and not (stm & 1 or etx_stop or restart)
        r = rx.comb(rx_go, u_data, rx_stop, pin_in, t["stick"], t["dtick"])
        a_e = etx.after(t["done"], 0, (1 << tx.dpin) & self.pins.m, tx.d2_mask, 0)
        a_r = erx.after(r["done"], rx.chain_on, 0, 0, (pin_in >> rx.ipin) & 1)

        # read data for the host
        st = self._reg_status(dm_rd)
        r_rdata = self.regs.read(h["r_addr"], st)

        rd_rx = e["rd_strobe"] and e["rd_unit"] == 1 and e["rd_idx"] == 0

        # ---- clock edge
        self.host.clock(rst, din, op, stb, r_rdata)
        self.imem.clock(rc["im_we"], rc["im_waddr"], rc["im_wdata"], self.ctrl.pc)
        if rc["dm_acc"]:
            self.dmem.clock(rc["dm_we"], rc["dm_addr"], rc["dm_wdata"])
        else:
            self.dmem.clock(e["dm_we"], e["dm_addr"], e["dm_wdata"])
        txf.clock(rst, rc["txf_wr"] or e["txf_wr"], rc["txf_wdata"] if rc["txf_wr"] else e["fdata"],
                  u_pop or chain_pop or e["txf_rd"], rc["txf_clr"])
        rxf.clock(rst, r["push"] or e["rxf_wr"], r["push_data"] if r["push"] else e["fdata"],
                  rc["rxf_rd"] or e["rxf_rd"], rc["rxf_clr"])
        etx_ack = te["start_req"] and tx_go
        erx_ack = tr["start_req"] and rx_go
        cw = e["cfg_we"]
        cu, ci, cd = e["cfg_unit"], e["cfg_idx"], e["cfg_data"]
        crc.clock(rst, cw and cu == 4, ci, cd, t["bit_v"], t["bit_o"], r["bit_v"], r["bit_o"], t["chain"] and next_crc)
        etx.clock(rst, restart, te, a_e, tx_busy, t["done"], int(bool(stm & 1 or etx_stop)), etx_ack,
                  e["det_clr"] and not e["det_sel"], e["det_arm"] and not e["det_sel"],
                  e["det_disarm"] and not e["det_sel"], cw and cu == 2, ci, cd)
        erx.clock(rst, restart, tr, a_r, rx.busy, r["done"], int(bool(stm & 2 or erx_stop)), erx_ack,
                  e["det_clr"] and e["det_sel"], e["det_arm"] and e["det_sel"],
                  e["det_disarm"] and e["det_sel"], cw and cu == 3, ci, cd)
        rxrdy_after = a_r["after_end"]
        rx.clock(rst, restart, cw and cu == 1, ci, cd, r, rxrdy_after, rd_rx, rc["rx_ovr_clr"])
        tx.clock(rst, cw and cu == 0, ci, cd, t)
        self.pins.clock(rst, (e["wm"], e["wv"], e["om"], e["ov"]),
                        (t["pw_mask"] | a_e["pw_mask"], (t["pw_val"] & t["pw_mask"]) | (a_e["pw_val"] & a_e["pw_mask"]),
                         a_e["po_mask"], a_e["po_val"]),
                        (r["wm"], r["wv"], 0, 0), tx_busy, rx_busy, self.regs.owner, rc["owner_clr"], uio_in,
                        self.regs.filt)
        self.ctrl.clock(rst, restart, self.regs.start_pc, rc["db_set"], e)
        self.regs.clock(rst, h, rc, e["ill"], e["stk"], underrun, e["host_irq"])
        self.rst_q = [1 - (rst_n & 1), self.rst_q[0]]
        self.cycle += 1
