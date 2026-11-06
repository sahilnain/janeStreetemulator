# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_pins: unit test for src/io/pe_pins.v (v2: three write ports).

What is checked
---------------
A Python model of docs/spec/blocks.md Section 7 runs alongside:
  owner 0/3 controller, 1 TX, 2 RX
  CTRL may write pin i if owner is controller, or the owner unit is idle
  TXP only TX-owned pins, RXP only RX-owned pins
  denied bits are dropped and set owner_err (sticky, clr_err clears, set wins)
  priority TXP > RXP > CTRL (only reachable with an idle unit writing)
  pad_out = out & ~od; pad_oe = od ? ~out : oe; pin_in = pad_in delayed 2.
  pin_filt[i] = 1: pin_in takes a new level after 4 samples in a row have it.
1. random_ports: 5000 cycles of random writes on all ports, random owners,
   busy flags, open-drain bits and filter bits.
2. reset_state: out = 1, oe = 0, no drive, owner_err = 0.
3. busy_rule: controller write to a TX/RX-owned pin succeeds while the unit
   is idle and is dropped (owner_err) while it is busy.
"""

import random

import cocotb
from tbutil import fall, param, settle, start, v

NP = param("NUM_PINS", 8)
M = (1 << NP) - 1
PORTS = ("c_wm", "c_wv", "c_om", "c_ov", "t_wm", "t_wv", "t_om", "t_ov", "r_wm", "r_wv")


class Model:
    def __init__(self):
        self.out, self.oe, self.err = M, 0, 0
        self.level, self.seen = 0, [0] * NP     # filter: the output, and differing samples in a row

    def pin_in(self, sync, filt):
        return (self.level & filt) | (sync & ~filt & M)

    def filter_step(self, sync, filt):
        for i in range(NP):
            b = 1 << i
            if not filt & b:
                self.level = (self.level & ~b) | (sync & b)
                self.seen[i] = 0
            elif (sync ^ self.level) & b:
                self.seen[i] += 1
                if self.seen[i] == 4:
                    self.level ^= b
                    self.seen[i] = 0
            else:
                self.seen[i] = 0

    def step(self, p, owner, od, txb, rxb, clr):
        own = [(owner >> (2 * i)) & 3 for i in range(NP)]
        ot = sum(1 << i for i in range(NP) if own[i] == 1)
        orx = sum(1 << i for i in range(NP) if own[i] == 2)
        oc = sum(1 << i for i in range(NP) if own[i] in (0, 3) or (own[i] == 1 and not txb)
                 or (own[i] == 2 and not rxb))
        twh, toh, rwh = p["t_wm"] & ot, p["t_om"] & ot, p["r_wm"] & orx
        cwh = p["c_wm"] & oc & ~twh & ~rwh
        coh = p["c_om"] & oc & ~toh
        hit = twh | rwh | cwh
        val = (p["t_wv"] & twh) | (p["r_wv"] & rwh) | (p["c_wv"] & cwh)
        ohit = toh | coh
        oval = (p["t_ov"] & toh) | (p["c_ov"] & coh)
        denied = (p["t_wm"] & ~ot) | (p["t_om"] & ~ot) | (p["r_wm"] & ~orx) | (p["c_wm"] & ~oc) | (p["c_om"] & ~oc)
        self.out = (self.out & ~hit | val) & M
        self.oe = (self.oe & ~ohit | oval) & M
        if denied & M:
            self.err = 1
        elif clr:
            self.err = 0


def pads(m, od):
    return m.out & ~od & M, ((od & ~m.out) | (~od & m.oe)) & M


async def init(dut):
    for s in PORTS + ("tx_busy", "rx_busy", "pin_owner", "pin_od", "pin_filt", "clr_err", "pad_in"):
        getattr(dut, s).value = 0
    await start(dut)
    return Model()


def check(dut, m, od):
    po, poe = pads(m, od)
    assert v(dut.pad_out) == po, f"pad_out {v(dut.pad_out):x} != {po:x}"
    assert v(dut.pad_oe) == poe, f"pad_oe {v(dut.pad_oe):x} != {poe:x}"
    assert v(dut.owner_err) == m.err


@cocotb.test(timeout_time=10, timeout_unit="ms")
async def random_ports(dut):
    m = await init(dut)
    hist = [0, 0]
    for n in range(5000):
        if n % 300 == 0:
            owner = random.getrandbits(2 * NP)
            od = random.getrandbits(NP) if random.random() < 0.5 else 0
            filt = random.getrandbits(NP) if random.random() < 0.5 else 0
        p = {k: (random.getrandbits(NP) if random.random() < 0.3 else 0) for k in PORTS}
        txb, rxb, clr = random.getrandbits(1), random.getrandbits(1), int(random.random() < 0.05)
        # mostly steady inputs, or the filter never sees 4 equal samples
        pin = random.getrandbits(NP) if random.random() < 0.1 else hist[-1]
        for k, x in p.items():
            getattr(dut, k).value = x
        dut.tx_busy.value, dut.rx_busy.value, dut.clr_err.value = txb, rxb, clr
        dut.pin_owner.value, dut.pin_od.value, dut.pad_in.value = owner, od, pin
        dut.pin_filt.value = filt
        await settle()
        check(dut, m, od)
        assert v(dut.pin_in) == m.pin_in(hist[-2], filt), "pin_in: delayed by 2 cycles, then the filter"
        m.step(p, owner, od, txb, rxb, clr)
        m.filter_step(hist[-2], filt)
        hist.append(pin)
        await fall(dut)


@cocotb.test()
async def filter_pulse_width(dut):
    """A filtered pin ignores a pulse of up to 3 samples and follows one of 4, 4 cycles later."""
    await init(dut)
    dut.pin_filt.value = 1
    dut.pad_in.value = M
    for _ in range(8):
        await fall(dut)
    await settle()
    assert v(dut.pin_in) & 1 == 1
    for width in (1, 2, 3, 4):
        dut.pad_in.value = M - 1
        seen = []
        for t in range(width + 12):
            if t == width:
                dut.pad_in.value = M
            await fall(dut)
            await settle()
            seen.append(v(dut.pin_in) & 1)
        if width < 4:
            assert seen == [1] * len(seen), f"a pulse of {width} went through: {seen}"
        else:
            assert 0 in seen, "a pulse of 4 samples was lost"
            assert seen.index(0) == 2 + 4 - 1, f"delay: {seen}"
        for _ in range(8):
            await fall(dut)


@cocotb.test()
async def reset_state(dut):
    m = await init(dut)
    await settle()
    assert v(dut.pad_oe) == 0 and v(dut.owner_err) == 0
    assert v(dut.pad_out) == M
    check(dut, m, 0)


@cocotb.test()
async def busy_rule(dut):
    m = await init(dut)
    owner = 1 | (2 << 2)                       # P0 TX, P1 RX, others controller
    dut.pin_owner.value = owner
    for unit, busy_sig in ((0, "tx_busy"), (1, "rx_busy")):
        bit = 1 << unit
        for busy in (0, 1):
            getattr(dut, busy_sig).value = busy
            dut.c_wm.value, dut.c_wv.value = bit, 0
            await fall(dut)
            dut.c_wm.value = 0
            await settle()
            if busy:
                assert v(dut.owner_err) == 1 and (v(dut.pad_out) & bit), "dropped while busy"
                dut.clr_err.value = 1
                await fall(dut)
                dut.clr_err.value = 0
                dut.c_wm.value, dut.c_wv.value = bit, bit    # restore (now idle)
                getattr(dut, busy_sig).value = 0
                await fall(dut)
                dut.c_wm.value = 0
            else:
                assert v(dut.owner_err) == 0 and not (v(dut.pad_out) & bit), "allowed while idle"
                dut.c_wm.value, dut.c_wv.value = bit, bit
                await fall(dut)
                dut.c_wm.value = 0
    await settle()
    assert v(dut.owner_err) == 0
    assert m.err == 0
