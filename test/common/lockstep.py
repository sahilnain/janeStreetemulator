# SPDX-License-Identifier: Apache-2.0
"""
lockstep: compares the RTL with the Python model (model/pe_model) every clock cycle.

What this file does
-------------------
`Lockstep(dut, prefix)` runs one model next to one RTL instance:
* at each rising edge it reads the inputs the RTL samples (ui_in, uio_in, rst_n) and
  steps the model with them;
* at the next falling edge, when everything has settled, it compares the chip outputs
  and the internal state. A difference fails the test with the cycle, the signal and
  both values.
The model's memories start as copies of the RTL's when the check is armed, since RTL
memories keep their contents across a reset. Unknown (X) bits are not compared.

`chip.py` attaches one to every chip when the environment variable LOCKSTEP is set, so
any top-level test can run in lockstep: make LOCKSTEP=1.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT / "model") not in sys.path:
    sys.path.insert(0, str(ROOT / "model"))

from cocotb.triggers import FallingEdge, RisingEdge  # noqa: E402
from pe_model import Chip, Config, ModelError  # noqa: E402

INSTANCES = {"": "user_project", "e1_": "emu1", "e2_": "emu2"}


def res(sig):
    """int value of an RTL signal, or None if it holds X/Z."""
    val = sig.value
    return int(val) if val.is_resolvable else None


class Lockstep:
    def __init__(self, dut, prefix="", cfg=None):
        self.dut = dut
        self.p = prefix
        self.cfg = cfg or Config()
        self.chip = Chip(self.cfg)
        self.armed = False
        self.cycles = 0
        self.compared = 0
        self.up = getattr(dut, INSTANCES[prefix])
        self.ui = getattr(dut, prefix + "ui_in")
        self.uo = getattr(dut, prefix + "uo_out")
        self.uio_in = getattr(dut, prefix + "uio_in")
        self.uio_out = getattr(dut, prefix + "uio_out")
        self.uio_oe = getattr(dut, prefix + "uio_oe")
        self.rst_n = getattr(dut, prefix + "rst_n")
        self.internal = self._internal() if hasattr(self.up, "u_ctrl") else []  # none at gate level

    def _internal(self):
        up = self.up
        c, tx, rx, etx, erx = up.u_ctrl, up.u_txu, up.u_rxu, up.u_etx, up.u_erx
        always = lambda m: True          # noqa: E731
        tx_on = lambda m: m.tx.busy      # noqa: E731
        rx_on = lambda m: m.rx.busy      # noqa: E731
        h = [
            ("ctrl.pc", c.pc, lambda m: m.ctrl.pc, always),
            ("ctrl.slot", c.slot, lambda m: m.ctrl.slot, always),
            ("ctrl.z", c.z, lambda m: m.ctrl.z, always),
            ("ctrl.c", c.c, lambda m: m.ctrl.c, always),
            ("ctrl.to", c.to, lambda m: m.ctrl.to, always),
            ("ctrl.db", c.db, lambda m: m.ctrl.db, always),
            ("ctrl.sp", c.sp, lambda m: m.ctrl.sp, always),
            ("ctrl.stall", c.stall, lambda m: m.ctrl.stall, always),
            ("ctrl.blocked", c.blocked, lambda m: m.ctrl.blocked, always),
            ("tx.busy", tx.busy, lambda m: m.tx.busy, always),
            ("tx.idx", tx.idx, lambda m: m.tx.idx, tx_on),
            ("tx.o", tx.o, lambda m: m.tx.o, tx_on),
            ("tx.sh", tx.sh, lambda m: m.tx.sh, tx_on),
            ("tx.crc_pkt", tx.crc_pkt, lambda m: m.tx.crc_pkt, tx_on),
            ("tx.eph", tx.eph, lambda m: m.tx.eph, tx_on),
            ("tx.ones", tx.ones, lambda m: m.tx.ones, tx_on),
            ("tx.sw", tx.sw, lambda m: m.tx.sw, tx_on),
            ("rx.busy", rx.busy, lambda m: m.rx.busy, always),
            ("rx.rxrdy", rx.rxrdy, lambda m: m.rx.rxrdy, always),
            ("rx.overrun", rx.overrun, lambda m: m.rx.overrun, always),
            ("rx.data_q", rx.data_q, lambda m: m.rx.data_q, always),
            ("rx.nbits", rx.nbits, lambda m: m.rx.nb, rx_on),
            ("rx.oidx", rx.oidx, lambda m: m.rx.oidx, lambda m: m.rx.busy and m.rx.need),
            ("rx.wins", rx.wins, lambda m: m.rx.wins, lambda m: m.rx.busy and not m.rx.tick & 3),
            ("rx.wcnt", rx.wcnt, lambda m: m.rx.wcnt, lambda m: m.rx.busy and m.rx.tick & 3 in (0, 3)),
            ("rx.hunting", rx.hunting, lambda m: m.rx.hunting, rx_on),
            ("rx.rones", rx.rones, lambda m: m.rx.rones, rx_on),
            ("rx.serr", rx.serr, lambda m: m.rx.serr, always),
            ("crc.crc", up.u_crc.crc, lambda m: m.crc.crc, always),
            ("crc.skip", up.u_crc.skip, lambda m: m.crc.skip, always),
            ("crc.sent", up.u_crc.sent, lambda m: m.crc.sent, always),
            ("txf.cnt", up.u_txf.cnt, lambda m: m.txf.cnt, always),
            ("rxf.cnt", up.u_rxf.cnt, lambda m: m.rxf.cnt, always),
            ("txf.overrun", up.u_txf.overrun, lambda m: m.txf.overrun, always),
            ("rxf.overrun", up.u_rxf.overrun, lambda m: m.rxf.overrun, always),
            ("pins.out_r", up.u_pins.out_r, lambda m: m.pins.out_r, always),
            ("pins.oe_r", up.u_pins.oe_r, lambda m: m.pins.oe_r, always),
            ("pins.owner_err", up.u_pins.owner_err, lambda m: m.pins.owner_err, always),
            ("pins.filt_out", up.u_pins.filt_out, lambda m: m.pins.f_out, always),
            ("pins.filt_c0", up.u_pins.filt_c0, lambda m: m.pins.f_c0, always),
            ("pins.filt_c1", up.u_pins.filt_c1, lambda m: m.pins.f_c1, always),
            ("regs.run", up.u_regs.run, lambda m: m.regs.run, always),
            ("regs.imaddr", up.u_regs.imaddr, lambda m: m.regs.imaddr, always),
            ("regs.start_pc", up.u_regs.start_pc, lambda m: m.regs.start_pc, always),
            ("regs.pin_owner", up.u_regs.pin_owner, lambda m: m.regs.owner, always),
            ("regs.pin_od", up.u_regs.pin_od, lambda m: m.regs.od, always),
            ("regs.pin_filt", up.u_regs.pin_filt, lambda m: m.regs.filt, always),
            ("regs.irq_en", up.u_regs.irq_en, lambda m: m.regs.irq_en, always),
            ("regs.imem_err", up.u_regs.imem_err, lambda m: m.regs.imem_err, always),
            ("regs.dm_addr", up.u_regs.dm_addr, lambda m: m.regs.dm_addr, always),
            ("regs.st2", up.u_regs.status2, lambda m: m.regs.status2(m.rx.overrun), always),
            ("tx.lvl", tx.lvl, lambda m: m.tx.lvl, tx_on),
            ("tx.lf", tx.lf, lambda m: m.tx.lf, tx_on),
            ("tx.bit", tx.c_bit, lambda m: m.tx.bit, always),
            ("rx.word", rx.sh, lambda m: m.rx.word, rx_on),
            ("rx.od", rx.od, lambda m: m.rx.od, lambda m: m.rx.busy and m.rx.need),
            ("rx.hz", rx.hz, lambda m: m.rx.hz, rx_on),
            ("rx.lp", rx.lp, lambda m: m.rx.sprev, rx_on),
            ("rx.bit", rx.c_bit, lambda m: m.rx.bit, always),
            ("rx.step", rx.c_step, lambda m: m.rx.step, always),
            ("rx.first", rx.c_first, lambda m: m.rx.first, always),
            ("vote.running", rx.u_vote.running, lambda m: m.rx.vote.running, always),
            ("vote.ones", rx.u_vote.ones, lambda m: m.rx.vote.ones, lambda m: m.rx.vote.running),
            ("crc.poly", up.u_crc.c_poly, lambda m: m.crc.poly, always),
            ("host.r_addr", up.u_host_if.r_addr, lambda m: m.host.addr, always),
            ("host.wphase", up.u_host_if.wphase, lambda m: m.host.wphase, always),
            ("host.rphase", up.u_host_if.rphase, lambda m: m.host.rphase, always),
        ]
        for i in range(8):
            h.append((f"ctrl.r{i}", c.r[i], lambda m, i=i: m.ctrl.r[i], always))
        for nm, d, get in (("etx", etx, lambda m: m.etx), ("erx", erx, lambda m: m.erx)):
            h += [
                (f"{nm}.armed", d.armed, lambda m, g=get: g(m).armed, always),
                (f"{nm}.c_cond", d.c_cond, lambda m, g=get: g(m).cond, always),
                (f"{nm}.c_dur", d.c_dur, lambda m, g=get: g(m).dur, always),
                (f"{nm}.c_after", d.c_after, lambda m, g=get: g(m).aft, always),
                (f"{nm}.c_adly", d.c_adly, lambda m, g=get: g(m).adly, always),
                (f"{nm}.pend", d.pend, lambda m, g=get: g(m).pend, always),
                (f"{nm}.mine", d.mine, lambda m, g=get: g(m).mine, always),
                (f"{nm}.flag", d.flag, lambda m, g=get: g(m).flag, always),
                (f"{nm}.ferr", d.ferr, lambda m, g=get: g(m).ferr, always),
                (f"{nm}.apend", d.apend, lambda m, g=get: g(m).apend, always),
                (f"{nm}.acnt", d.acnt, lambda m, g=get: g(m).acnt, lambda m, g=get: g(m).apend),
                (f"{nm}.cnt", d.cnt, lambda m, g=get: g(m).cnt, lambda m, g=get: g(m).armed),
            ]
        return h

    def stop(self):
        """End the check and the tasks of this run (test_lockstep runs several on one reset chain)."""
        for t in (getattr(self, "task", None), getattr(self, "driver", None)):
            if t is not None:
                t.cancel()

    def _copy_mem(self, handle, mem):
        for i in range(len(mem)):
            mem[i] = res(handle[i])

    def arm(self):
        """Start comparing: the reset is over and both sides are in reset state."""
        up, m = self.up, self.chip
        if self.internal:
            self._copy_mem(up.u_imem.mem, m.imem.mem)
            self._copy_mem(up.u_dmem.mem, m.dmem.mem)
            self._copy_mem(up.u_txf.mem, m.txf.mem)
            self._copy_mem(up.u_rxf.mem, m.rxf.mem)
            m.imem.rdata = res(up.u_imem.rdata)
        self.armed = True

    def _context(self):
        """The model's unit state, to help read a failure."""
        m = self.chip
        return (f"pin_in={m.pins.pin_in(m.regs.filt):02x} tx busy={m.tx.busy} pins={m.tx.pins:04x} bit={m.tx.bit} frame={m.tx.frame:04x} "
                f"clk={m.tx.clk} tick={m.tx.tick:04x} line={m.tx.line:04x} | rx busy={m.rx.busy} pins={m.rx.pins:04x} "
                f"bit={m.rx.bit} frame={m.rx.frame:04x} vote={m.rx.vcfg:02x} step={m.rx.step} first={m.rx.first} "
                f"tick={m.rx.tick:04x} odly={m.rx.odly:04x} | etx cond={m.etx.cond:04x} dur={m.etx.dur} aft={m.etx.aft} "
                f"| erx cond={m.erx.cond:04x} dur={m.erx.dur} aft={m.erx.aft} adly={m.erx.adly}")

    def _fail(self, name, rtl, mod):
        raise AssertionError(f"lockstep {self.p}cycle {self.cycles}: {name} RTL={rtl!r} model={mod!r}\n{self._context()}")

    def compare(self):
        o = self.chip.outputs()
        known = o["uo_known"]
        rtl_bits = str(self.uo.value)                     # MSB first, may contain X
        for i in range(8):
            if (known >> i) & 1:
                rb = rtl_bits[7 - i]
                if rb not in "01" or int(rb) != (o["uo_out"] >> i) & 1:
                    self._fail(f"uo_out[{i}]", self.uo.value, f"{o['uo_out']:08b} (known mask {known:08b})")
        for name, sig, key in (("uio_out", self.uio_out, "uio_out"), ("uio_oe", self.uio_oe, "uio_oe")):
            r = res(sig)
            if r != o[key]:
                self._fail(name, sig.value, f"{o[key]:08b}")
        m = self.chip
        for name, sig, get, when in self.internal:
            if not when(m):
                continue
            mv = get(m)
            if mv is None:
                continue
            r = res(sig)
            if r != int(mv):
                self._fail(name, sig.value, mv)
        self.compared += 1

    async def run(self):
        clk = self.dut.clk
        while True:
            await RisingEdge(clk)
            try:
                if self.rst_n.value.is_resolvable and self.ui.value.is_resolvable and self.uio_in.value.is_resolvable:
                    self.chip.step(int(self.ui.value), int(self.uio_in.value), int(self.rst_n.value))
                else:                                     # before the bench has set the inputs
                    self.chip.step(0, 0xFF, 0)
            except ModelError as err:                     # the test runs an unwritten IMEM word
                self.dut._log.warning(f"lockstep stopped at cycle {self.cycles}: {err}")
                return
            self.cycles += 1
            await FallingEdge(clk)
            if self.armed:
                self.compare()
