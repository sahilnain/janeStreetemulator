# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_det: unit test for src/unit/pe_det.v (event detector, ETX / ERX).

What is checked
---------------
1. random_model: 20000 cycles against a cycle model of docs/spec/blocks.md
   Section 5, with random pins, config writes, arm/disarm/clr commands and
   a fake unit (busy for a random time after an accepted start, done in its
   last busy cycle; also unit starts not caused by the detector, and stops).
   Every cycle: start_req, flag, ferr, after_pend, after_end, drive outputs,
   status register.
Directed (absolute cycle numbers, from the spec text):
2. level_exact: pin low from cycle p -> fires in p + DUR - 1 (DUR 0/1 -> p).
3. edge_modes: falling / rising / any; armed on a low line never fires on
   falling; an edge reverted before DUR cancels; any-edge restarts on a
   new edge.
4. qualifier: level counts only while QPIN == QLVL; an edge needs the
   qualifier in the edge cycle (I2C START/STOP).
5. arm_pending_autoarm: arm while the unit is busy -> pending -> armed the
   cycle after after_end; AUTOARM only after its own start; disarm clears.
6. after_actions: ETX drive 0 / drive 1 / release in D; ERX check at
   D + ADLY (and ADLY = 0) sets FERR, after_pend covers D+1..D+ADLY; stop
   cancels a pending check.
7. cfg_resets_counter: a COND/DUR write while counting restarts the count.
8. stop_and_release2: STOP gives stop_req in the fire cycle only; release covers D2.
9. no_check_in_chain: found by formal: with the unit in CHAIN a done starts no check.
"""

import random

import cocotb
import pe_isa as I
from tbutil import fall, param, settle, start, v

KIND = param("KIND", 0)
NP = param("NUM_PINS", 8)
INS = ("cfg_we", "cfg_idx", "cfg_data", "clr", "arm", "disarm", "restart", "pin_in",
       "unit_busy", "unit_done", "unit_stop", "start_ack", "act_pin", "act2_mask", "unit_chain")


class Model:
    def __init__(self):
        self.cond = self.dur = self.after = self.adly = 0
        self.armed = self.pend = self.mine = self.prev = self.cnt = self.hold = self.hlvl = 0
        self.apend = self.acnt = self.flag = self.ferr = 0

    def f(self, name, lo, w):
        return (self.cond >> lo) & ((1 << w) - 1)

    def comb(self, i):
        pin = lambda p: (i["pin_in"] >> p) & 1 if p < NP else 0  # noqa: E731
        cur = pin(self.f("", 0, 4))
        mode, lvl = self.f("", 4, 2), self.f("", 6, 1)
        qual = (not self.f("", 13, 1)) or pin(self.f("", 9, 4)) == self.f("", 14, 1)
        dure = self.dur or 1
        if mode == 0:
            edge, c, n = 0, cur == lvl and qual, self.cnt + 1
        else:
            dir_ok = mode == 3 or (mode == 1 and cur) or (mode == 2 and not cur)
            edge = int(cur != self.prev and dir_ok and qual)
            c = edge or (self.hold and cur == self.hlvl)
            n = 1 if edge else self.cnt + 1
        fire = int(self.armed and c and n == dure)
        chk_en = KIND == 1 and (self.after & 1)
        done = i["unit_done"]
        aend = int((done and not (chk_en and self.adly)) or (self.apend and self.acnt == 1))
        chk_now = chk_en and ((done and self.adly == 0) or (self.apend and self.acnt == 1))
        am = (1 << i["act_pin"]) & ((1 << NP) - 1)
        drv = KIND == 0 and done and self.after != 0
        drv_out = drv and self.after != 3
        o = dict(start_req=int(fire and self.f("", 7, 1)), after_end=aend, after_pend=self.apend,
                 flag=self.flag, ferr=self.ferr,
                 pw_mask=am if drv_out else 0, pw_val=am if drv_out and self.after == 2 else 0,
                 po_mask=am if drv else 0, po_val=am if drv_out else 0,
                 rd_data=(self.ferr << 4) | ((self.mine | self.apend) << 3) | (self.pend << 2) |
                 (self.flag << 1) | self.armed)
        return o, dict(cur=cur, c=c, n=n, edge=edge, fire=fire, aend=aend, chk_now=chk_now,
                       act=pin(i["act_pin"]))

    def step(self, i):
        o, x = self.comb(i)
        if i["restart"]:
            self.armed = self.pend = self.mine = self.flag = self.ferr = 0
            self.hold = self.hlvl = self.cnt = self.apend = self.acnt = 0
            self.prev = x["cur"]
        else:
            cfg_trig = i["cfg_we"] and i["cfg_idx"] in (0, 1)
            if not self.armed or x["fire"] or cfg_trig or not x["c"]:
                self.cnt, self.hold = 0, 0
            else:
                self.cnt = x["n"] & 0xFFFF
                if x["edge"]:
                    self.hold, self.hlvl = 1, x["cur"]
            self.prev = x["cur"]
            if x["fire"]:
                self.flag = 1
            elif i["clr"]:
                self.flag = 0
            active = i["unit_busy"] or self.apend
            rearm = x["aend"] and (self.pend or (self.f("", 8, 1) and self.mine))
            idle_rel = self.pend and not active and not i["unit_done"]
            if x["fire"] or i["disarm"]:
                armed = 0
            elif rearm or (i["arm"] and not active) or idle_rel:
                armed = 1
            else:
                armed = self.armed
            if i["disarm"] or rearm:
                pend = 0
            elif i["arm"] and active:
                pend = 1
            elif idle_rel:
                pend = 0
            else:
                pend = self.pend
            self.armed, self.pend = armed, pend
            if i["start_ack"]:
                self.mine = 1
            elif x["aend"] or i["unit_stop"]:
                self.mine = 0
            chk_en = KIND == 1 and (self.after & 1)
            if i["unit_stop"]:
                self.apend = 0
            elif i["unit_done"] and chk_en and self.adly:
                self.apend, self.acnt = 1, self.adly
            elif self.apend:
                if self.acnt == 1:
                    self.apend = 0
                else:
                    self.acnt -= 1
            if x["chk_now"]:
                self.ferr = int(x["act"] != ((self.after >> 1) & 1))
        if i["cfg_we"]:
            idx, d = i["cfg_idx"], i["cfg_data"]
            if idx == 0:
                self.cond = d & 0x7FFF
            elif idx == 1:
                self.dur = d
            elif idx == 2:
                self.after = d & 3
            elif idx == 3:
                self.adly = d
        return o


def zero():
    return {k: 0 for k in INS}


async def drive(dut, i):
    for k, x in i.items():
        getattr(dut, k).value = x
    await settle()


def check(dut, o, where=""):
    for k, e in o.items():
        g = v(getattr(dut, k))
        assert g == e, f"{where}: {k} = {g}, expected {e}"


async def init(dut):
    for s in INS:
        getattr(dut, s).value = 0
    await start(dut)


@cocotb.test(timeout_time=100, timeout_unit="ms")
async def random_model(dut):
    await init(dut)
    m = Model()
    left = 0             # fake unit: cycles of busy left
    pins = 0
    act = 0
    for t in range(20000):
        i = zero()
        if random.random() < 0.15:
            pins ^= 1 << random.randrange(NP)
        i["pin_in"] = pins
        if t % 500 == 0:
            act = random.randrange(NP)
        i["act_pin"] = act
        r = random.random()
        if r < 0.02:
            i["cfg_we"], i["cfg_idx"] = 1, random.randrange(5)
            i["cfg_data"] = random.getrandbits(16)
            if i["cfg_idx"] == 0:
                i["cfg_data"] = (random.getrandbits(15) & ~0xF) | random.randrange(NP)
            elif i["cfg_idx"] in (1, 3):
                i["cfg_data"] = random.choice([0, 1, 2, 3, random.randint(4, 30)])
        elif r < 0.06:
            i["arm"] = 1
        elif r < 0.07:
            i["disarm"] = 1
        elif r < 0.09:
            i["clr"] = 1
        elif r < 0.0905:
            i["restart"] = 1
        # fake unit: busy = left > 0; done in the last busy cycle
        i["unit_busy"] = int(left > 0)
        i["unit_done"] = int(left == 1)
        if left > 0 and random.random() < 0.003:
            i["unit_stop"], i["unit_done"] = 1, 0
        o, _ = m.comb(i)
        accept = o["start_req"] and not (left > 0 or m.apend)
        i["start_ack"] = int(accept)
        await drive(dut, i)
        check(dut, o, f"t={t}")
        m.step(i)
        if i["unit_stop"]:
            left = 0
        elif left > 0:
            left -= 1
        elif accept or (random.random() < 0.01 and not m.apend):
            left = random.randint(1, 25)      # started by the detector, or by ustart
        await fall(dut)


def idle(dut):
    """All inputs 0 except pin_in, which keeps its level (no fake edges)."""
    i = zero()
    i["pin_in"] = v(dut.pin_in)
    return i


async def cfg(dut, idx, data):
    i = idle(dut)
    i.update(cfg_we=1, cfg_idx=idx, cfg_data=data)
    await drive(dut, i)
    await fall(dut)
    await drive(dut, idle(dut))


async def cmd(dut, name):
    i = idle(dut)
    i[name] = 1
    await drive(dut, i)
    await fall(dut)
    await drive(dut, idle(dut))


async def pulse_until(dut, n, pins_fn, sig="start_req"):
    """Run n cycles with pin_in = pins_fn(t); return cycles where `sig` was 1."""
    hits = []
    for t in range(n):
        dut.pin_in.value = pins_fn(t)
        await settle()
        if v(getattr(dut, sig)):
            hits.append(t)
        await fall(dut)
    return hits


@cocotb.test()
async def level_exact(dut):
    await init(dut)
    for dur in (0, 1, 2, 5, 217):
        dut.pin_in.value = 0xFF
        await cfg(dut, I.D_COND, I.cond(3, I.LEVEL, 0, start=1))
        await cfg(dut, I.D_DUR, dur)
        await cmd(dut, "arm")
        p = 10
        hits = await pulse_until(dut, p + 260, lambda t: 0xFF & ~(8 if t >= p else 0))
        assert hits == [p + max(dur, 1) - 1], (dur, hits)
        assert v(dut.flag) == 1
        await cmd(dut, "clr")
        assert v(dut.flag) == 0


@cocotb.test()
async def edge_modes(dut):
    await init(dut)
    # falling, armed on a low line: no fire until a real edge
    dut.pin_in.value = 0
    await cfg(dut, I.D_COND, I.cond(2, I.FALL, start=1))
    await cfg(dut, I.D_DUR, 4)
    await cmd(dut, "arm")
    hits = await pulse_until(dut, 30, lambda t: 0 if t < 10 else (4 if t < 20 else 0))
    assert hits == [20 + 3], hits
    # edge reverted before DUR: cancelled; next good edge fires
    await cmd(dut, "arm")
    wave = lambda t: 4 if t < 5 or 7 <= t < 12 else 0   # noqa: E731  low 5..6 (2 cyc), low from 12
    hits = await pulse_until(dut, 30, wave)
    assert hits == [12 + 3], hits
    # rising
    await cfg(dut, I.D_COND, I.cond(2, I.RISE, start=1))
    await cmd(dut, "arm")
    hits = await pulse_until(dut, 30, lambda t: 4 if t >= 8 else 0)
    assert hits == [8 + 3], hits
    # any edge, restart counting on each new edge
    await cfg(dut, I.D_COND, I.cond(2, I.ANY, start=1))
    await cmd(dut, "arm")
    wave = lambda t: 4 if t < 6 else (0 if t < 8 else 4)   # noqa: E731  edges at 6 and 8
    hits = await pulse_until(dut, 30, wave)
    assert hits == [8 + 3], hits


@cocotb.test()
async def qualifier(dut):
    await init(dut)
    SDA, SCL = 1, 4
    # I2C START/STOP: any edge of SDA while SCL = 1, DUR = 1
    await cfg(dut, I.D_COND, I.cond(SDA, I.ANY, start=1, qpin=SCL, qen=1, qlvl=1))
    await cfg(dut, I.D_DUR, 1)
    hi = 1 << SCL
    sda = 1 << SDA
    for scl_level, expect_fire in ((0, False), (1, True)):
        dut.pin_in.value = sda | (hi if scl_level else 0)
        await fall(dut, 2)
        await cmd(dut, "arm")
        hits = await pulse_until(dut, 10, lambda t: (sda if t < 5 else 0) | (hi if scl_level else 0))
        assert (hits == [5]) == expect_fire, (scl_level, hits)
        await cmd(dut, "disarm")
    # level mode: counts only while the qualifier holds
    await cfg(dut, I.D_COND, I.cond(SDA, I.LEVEL, 0, start=1, qpin=SCL, qen=1, qlvl=1))
    await cfg(dut, I.D_DUR, 5)
    await cmd(dut, "arm")
    wave = lambda t: (hi if 3 <= t < 6 or t >= 10 else 0)   # noqa: E731  SDA low all the time
    hits = await pulse_until(dut, 30, wave)
    assert hits == [10 + 4], hits


async def unit_packet(dut, L, ack=0, stop_at=None):
    """Fake unit packet of L busy cycles (done in the last); returns after_end cycles."""
    ends = []
    for t in range(L + 40):
        i = zero()
        i.update(pin_in=0, unit_busy=int(t < L), unit_done=int(t == L - 1), start_ack=0,
                 act_pin=0)
        if stop_at is not None and t == stop_at:
            i["unit_stop"] = 1
        await drive(dut, i)
        if v(dut.after_end):
            ends.append(t)
        await fall(dut)
    return ends


@cocotb.test()
async def arm_pending_autoarm(dut):
    await init(dut)
    await cfg(dut, I.D_COND, I.cond(0, I.LEVEL, 1, start=1, autoarm=1))
    await cfg(dut, I.D_DUR, 3)
    # arm while busy -> pending -> armed the cycle after after_end
    i = zero()
    i.update(unit_busy=1, arm=1)
    await drive(dut, i)
    await fall(dut)
    i = zero()
    i.update(unit_busy=1)
    await drive(dut, i)
    assert v(dut.rd_data) & 0b101 == 0b100, "pending, not armed"
    await fall(dut)
    i = zero()
    i.update(unit_busy=1, unit_done=1)
    await drive(dut, i)
    assert v(dut.after_end) == 1
    await fall(dut)
    await drive(dut, zero())
    assert v(dut.rd_data) & 0b101 == 0b001, "armed after after_end"
    # fires (pin 0 low != LVL 1? LVL=1, pin low -> no). Make it fire with pin high:
    i = zero()
    i["pin_in"] = 1
    for t in range(3):
        await drive(dut, i)
        if t == 2:
            assert v(dut.start_req) == 1
            i2 = dict(i)
            i2["start_ack"] = 1
            await drive(dut, i2)
        await fall(dut)
    assert v(dut.rd_data) & 1 == 0, "disarmed by firing"
    # its own packet ends: AUTOARM re-arms
    await unit_packet(dut, 5)
    await settle()
    assert v(dut.rd_data) & 1 == 1, "AUTOARM after own packet"
    # a packet it did not start: no AUTOARM
    await cmd(dut, "disarm")
    await unit_packet(dut, 5)
    await settle()
    assert v(dut.rd_data) & 1 == 0, "no AUTOARM after a foreign packet"
    # disarm clears pending
    i = zero()
    i.update(unit_busy=1, arm=1)
    await drive(dut, i)
    await fall(dut)
    i = zero()
    i.update(unit_busy=1, disarm=1)
    await drive(dut, i)
    await fall(dut)
    await unit_packet(dut, 3)
    await settle()
    assert v(dut.rd_data) & 0b101 == 0
    # pending + the unit stopped (no done): arms once the unit is idle
    i = zero()
    i.update(unit_busy=1, arm=1)
    await drive(dut, i)
    await fall(dut)
    await drive(dut, zero())
    await fall(dut)
    await settle()
    assert v(dut.rd_data) & 0b101 == 0b001


@cocotb.test()
async def after_actions(dut):
    await init(dut)
    if KIND == 0:
        for after, (wm, wv, om, ov) in ((I.A_DRIVE0, (1, 0, 1, 1)), (I.A_DRIVE1, (1, 1, 1, 1)),
                                        (I.A_RELEASE, (0, 0, 1, 0)), (I.A_NONE, (0, 0, 0, 0))):
            await cfg(dut, I.D_AFTER, after)
            i = zero()
            i.update(unit_busy=1, unit_done=1, act_pin=5)
            await drive(dut, i)
            b = 1 << 5
            got = (v(dut.pw_mask), v(dut.pw_val), v(dut.po_mask), v(dut.po_val))
            assert got == (wm * b, wv * b, om * b, ov * b), (after, got)
            assert v(dut.after_end) == 1 and v(dut.after_pend) == 0
            await fall(dut)
            await drive(dut, zero())
            assert v(dut.pw_mask) == 0 and v(dut.po_mask) == 0
    else:
        for adly in (0, 1, 7):
            for lvl_pin in (0, 1):
                await cfg(dut, I.D_AFTER, I.erx_after(1, 1))
                await cfg(dut, I.D_ADLY, adly)
                ends, pend = [], []
                for t in range(12):
                    i = zero()
                    i.update(unit_busy=int(t == 0), unit_done=int(t == 0), act_pin=3,
                             pin_in=(lvl_pin << 3) if t == adly else ((1 - lvl_pin) << 3))
                    await drive(dut, i)
                    ends += [t] if v(dut.after_end) else []
                    pend += [t] if v(dut.after_pend) else []
                    await fall(dut)
                assert ends == [adly], (adly, ends)
                assert pend == list(range(1, adly + 1)), (adly, pend)
                assert v(dut.ferr) == int(lvl_pin != 1), "FERR from the sample at D + ADLY"
        # stop cancels a pending check
        await cfg(dut, I.D_ADLY, 5)
        ends = await unit_packet(dut, 1, stop_at=3)
        assert ends == [], ends
        assert v(dut.after_pend) == 0
        # CHECK off: after_end in D, FERR unchanged
        await cfg(dut, I.D_AFTER, 0)
        ferr = v(dut.ferr)
        ends = await unit_packet(dut, 4)
        assert ends == [3] and v(dut.ferr) == ferr


@cocotb.test()
async def cfg_resets_counter(dut):
    await init(dut)
    await cfg(dut, I.D_COND, I.cond(1, I.LEVEL, 0, start=1))
    await cfg(dut, I.D_DUR, 6)
    await cmd(dut, "arm")
    hits = []
    for t in range(20):
        i = zero()
        i["pin_in"] = 0
        if t == 4:
            i.update(cfg_we=1, cfg_idx=I.D_DUR, cfg_data=6)
        await drive(dut, i)
        hits += [t] if v(dut.start_req) else []
        await fall(dut)
    assert hits == [5 + 5], hits      # count restarts in cycle 5


@cocotb.test(timeout_time=200, timeout_unit="ms")
async def extremes(dut):
    """DUR / ADLY = 0xFFFF, out-of-range pins, every config bit."""
    from cocotb.triggers import ClockCycles
    await init(dut)
    await cfg(dut, I.D_COND, I.cond(NP - 1, I.LEVEL, 1, start=1))
    await cfg(dut, I.D_DUR, 0xFFFF)
    await cmd(dut, "arm")
    dut.pin_in.value = 1 << (NP - 1)
    await ClockCycles(dut.clk, 0xFFFF - 1, rising=False)
    await settle()
    assert v(dut.start_req) == 1, "fires in the 65535th cycle"
    await fall(dut)
    await settle()
    assert v(dut.start_req) == 0 and v(dut.flag) == 1
    if KIND == 1:
        await cfg(dut, I.D_AFTER, I.erx_after(1, 0))
        await cfg(dut, I.D_ADLY, 0xFFFF)
        i = idle(dut)
        i.update(unit_busy=1, unit_done=1, act_pin=NP - 1)
        await drive(dut, i)
        await fall(dut)
        i = idle(dut)
        i["act_pin"] = NP - 1
        await drive(dut, i)
        await ClockCycles(dut.clk, 0xFFFF - 1, rising=False)
        await settle()
        assert v(dut.after_end) == 1
        await fall(dut)
        await settle()
        assert v(dut.ferr) == 1 and v(dut.after_pend) == 0
    else:
        for p in range(NP):
            await cfg(dut, I.D_AFTER, I.A_DRIVE1)
            i = idle(dut)
            i.update(unit_busy=1, unit_done=1, act_pin=p)
            await drive(dut, i)
            assert v(dut.pw_mask) == 1 << p and v(dut.po_mask) == 1 << p
            await fall(dut)
    if NP < 16:                                # pins >= NUM_PINS read 0
        await cfg(dut, I.D_COND, I.cond(15, I.LEVEL, 0, start=1, qpin=14, qen=1, qlvl=0))
        await cfg(dut, I.D_DUR, 1)
        await cmd(dut, "arm")
        await settle()
        assert v(dut.start_req) == 1
    for idx in range(4):
        for b in range(16):
            await cfg(dut, idx, 1 << b)
        await cfg(dut, idx, 0)


@cocotb.test()
async def test_stop_and_release2(dut):
    """STOP: stop_req only in the fire cycle. Release also covers the second pin (TX DIFF)."""
    for s in INS:
        getattr(dut, s).value = 0
    await start(dut)
    dut.pin_in.value = 1 << 2
    await fall(dut, 2)
    dut.cfg_we.value, dut.cfg_idx.value, dut.cfg_data.value = 1, I.D_COND, I.cond(2, I.LEVEL, 0, stop=1)
    await fall(dut)
    dut.cfg_idx.value, dut.cfg_data.value = I.D_DUR, 2
    await fall(dut)
    dut.cfg_we.value = 0
    dut.arm.value = 1
    await fall(dut)
    dut.arm.value = 0
    await settle()
    assert not v(dut.stop_req)
    dut.pin_in.value = 0
    await settle()
    assert not v(dut.stop_req), "first cycle of DUR 2"
    await fall(dut)
    await settle()
    assert v(dut.stop_req) and not v(dut.start_req)
    await fall(dut)
    await settle()
    assert not v(dut.stop_req)
    if KIND == 0:
        dut.act_pin.value, dut.act2_mask.value = 1, 0b1000
        dut.cfg_we.value, dut.cfg_idx.value, dut.cfg_data.value = 1, I.D_AFTER, I.A_RELEASE
        await fall(dut)
        dut.cfg_we.value = 0
        dut.unit_busy.value, dut.unit_done.value = 1, 1
        await settle()
        assert v(dut.po_mask) == 0b1010 and v(dut.po_val) == 0
        assert v(dut.pw_mask) == 0
        await fall(dut)
        dut.unit_busy.value, dut.unit_done.value = 0, 0


@cocotb.test()
async def test_no_check_in_chain(dut):
    """Found by the chip proof: CHECK with RX CHAIN left the step pending while busy."""
    for s in INS:
        getattr(dut, s).value = 0
    await start(dut)
    for idx, val in ((I.D_AFTER, I.erx_after(1, 1)), (I.D_ADLY, 5)):
        dut.cfg_we.value, dut.cfg_idx.value, dut.cfg_data.value = 1, idx, val
        await fall(dut)
    dut.cfg_we.value = 0
    dut.unit_chain.value = 1
    dut.unit_busy.value, dut.unit_done.value = 1, 1
    await settle()
    assert v(dut.after_end), "a chained word ends its step at once"
    await fall(dut)
    dut.unit_done.value = 0
    await settle()
    assert not v(dut.after_pend)
    dut.unit_busy.value, dut.unit_chain.value = 0, 0
    await fall(dut)
