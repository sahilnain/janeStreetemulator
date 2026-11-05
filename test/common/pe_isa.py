# SPDX-License-Identifier: Apache-2.0
"""
pe_isa: the v2 instruction set, for the tests (docs/spec/isa.md).

What this file does
-------------------
* Encoders for the 32-bit word and every instruction.
* `R(n)`: a register operand. An int operand is an immediate (I = 1); an
  R operand uses I = 0 and rs = imm[2:0] ("op2").
* `Asm`: a tiny assembler with labels. Each instruction method appends one
  word; jump targets may be label names, resolved by `words()`.
* Config-register field helpers for the units and detectors
  (docs/spec/blocks.md Sections 3-5).

Word: [31:26] opcode [25:23] dly [22:20] A [19] I [18:16] B [15:0] imm16
"""

OP = dict(nop=0, jmp=1, djnz=2, call=3, ret=4, delay=6, mov=7, add=8, sub=9, and_=10,
          or_=11, xor=12, shl=13, shr=14, cmp=15, set=16, setoe=17, wait=18, jpin=19,
          rdpins=20, bitin=21, bitout=22, fget=23, fput=24, ld=25, st=26, ucfg=27, urd=28,
          ustart=29, ustop=30, uwait=31, evt=32, host=33, par=34)
OP_NAMES = {v: k for k, v in OP.items()}
RESERVED_OPS = [5] + list(range(35, 64))

# jump conditions
(ALWAYS, Z, NZ, C, NC, TO, NTO, ERX, NERX, TXU_BUSY, TXU_IDLE, RXU_BUSY, RXU_IDLE, ETX, NETX,
 FERR, NFERR, TXF_EMPTY, TXF_NEMPTY, TXF_FULL, RXF_EMPTY, RXF_NEMPTY, RXF_FULL, DB, NDB,
 RXRDY, NRXRDY) = range(27)

# set sources
SRC_0, SRC_1, SRC_C, SRC_NC = range(4)

# units
TX, RX, ETX_U, ERX_U, CRC = range(5)
M_TX, M_RX, M_BOTH = 1, 2, 3
FIFO_TX, FIFO_RX = 0, 1

# evt sub-ops
EV_WAIT, EV_WAITC, EV_CLR, EV_ARM, EV_DISARM = range(5)
DET_TX, DET_RX = 0, 1

# config register indices
TX_PINS, TX_BIT, TX_FRAME, TX_CLK, TX_TICK, TX_LINE = range(6)
RX_PINS, RX_BIT, RX_FRAME, RX_VOTE, RX_STEP, RX_FIRST, RX_TICK, RX_ODLY = range(8)
D_COND, D_DUR, D_AFTER, D_ADLY = range(4)
RX_DATA, RX_STAT = 0, 1
CRC_CTRL, CRC_POLY, CRC_INIT = range(3)
CRC_VAL, CRC_STAT = 0, 1
CRC_OFF, CRC_TX, CRC_RX = range(3)
RX_ST_SERR = 1 << 4
RX_ST_NBITS = 0x1F << 8

# detector modes
LEVEL, RISE, FALL, ANY = range(4)
# ETX after-packet actions
A_NONE, A_DRIVE0, A_DRIVE1, A_RELEASE = range(4)
# RX tick sources
SRC_OWN, SRC_TXTICK, SRC_EXT, SRC_DPLL = range(4)


class R:
    def __init__(self, n):
        assert 0 <= n <= 7
        self.n = n


def word(op, a=0, i=0, b=0, imm=0, dly=0):
    assert 0 <= dly <= 7 and 0 <= a <= 7 and 0 <= b <= 7 and 0 <= imm <= 0xFFFF
    return (op << 26) | (dly << 23) | (a << 20) | (i << 19) | (b << 16) | imm


def _op2(x):
    """(I, imm) for an op2 operand."""
    if isinstance(x, R):
        return 0, x.n
    assert 0 <= x <= 0xFFFF, x
    return 1, x


# ---------------------------------------------------------- config fields
def tx_pins(dpin, den=1, cpin=0, cen=0):
    return dpin | (den << 4) | (cpin << 8) | (cen << 12)


def tx_clk(cpol=0, cpha=0, stretch=0):
    return cpol | (cpha << 1) | (stretch << 2)


def tx_tick(ext=0, epin=0, eedge=0, lead=0):
    return ext | (epin << 4) | (eedge << 8) | (lead << 9)


def rx_pins(ipin, opin=0, oen=0, i2pin=0, diff=0):
    return ipin | (i2pin << 4) | (opin << 8) | (oen << 12) | (diff << 13)


def rx_frame(msb=0, hold=0, inv=0, chain=0, nbits=0):
    return msb | (hold << 1) | (inv << 2) | (chain << 3) | (nbits << 4)


def tx_frame(msb=0, inv=0, nbits=0, chain=0):
    return msb | (inv << 2) | (nbits << 4) | (chain << 9)


def tx_line(nrzi=0, eop=0, diff=0, d2pin=0, stuffn=0):
    return nrzi | (eop << 1) | (diff << 2) | (d2pin << 4) | (stuffn << 8)


def rx_line(odly=0, nrzi=0, hunt=0, stuffn=0):
    return odly | (nrzi << 4) | (hunt << 5) | (stuffn << 8)


def crc_ctrl(src=CRC_OFF, append=0, skip=0):
    return src | (append << 2) | (skip << 8)


def rx_vote(n, k=0):
    return n | (k << 4)


def rx_tick(src=SRC_OWN, epin=0, eedge=0, sdly=0):
    return src | (epin << 4) | (eedge << 8) | (sdly << 12)


def cond(pin, mode=LEVEL, lvl=0, start=0, autoarm=0, qpin=0, qen=0, qlvl=0, stop=0):
    return (pin | (mode << 4) | (lvl << 6) | (start << 7) | (autoarm << 8) | (qpin << 9) |
            (qen << 13) | (qlvl << 14) | (stop << 15))


def erx_after(check=1, lvl=1):
    return check | (lvl << 1)


# -------------------------------------------------------------- assembler
class Asm:
    def __init__(self, base=0):
        self.base = base
        self.items = []     # (fn(labels) -> word)
        self.labels = {}

    def L(self, name):
        assert name not in self.labels, name
        self.labels[name] = self.base + len(self.items)
        return self

    def here(self):
        return self.base + len(self.items)

    def raw(self, w):
        self.items.append(lambda lab, w=w: w)
        return self

    def _t(self, lab, t):
        return lab[t] if isinstance(t, str) else t

    def words(self):
        return [f(self.labels) for f in self.items]

    def _e(self, f):
        self.items.append(f)
        return self

    # flow
    def nop(self, dly=0):
        return self.raw(word(OP["nop"], dly=dly))

    def jmp(self, t, c=ALWAYS, dly=0):
        return self._e(lambda lab: word(OP["jmp"], imm=(c << 11) | self._t(lab, t), dly=dly))

    def djnz(self, rd, t, dly=0):
        return self._e(lambda lab: word(OP["djnz"], a=rd, imm=self._t(lab, t), dly=dly))

    def call(self, t, dly=0):
        return self._e(lambda lab: word(OP["call"], imm=self._t(lab, t), dly=dly))

    def ret(self, dly=0):
        return self.raw(word(OP["ret"], dly=dly))

    def delay(self, x, dly=0):
        i, v = _op2(x)
        return self.raw(word(OP["delay"], i=i, imm=v, dly=dly))

    # ALU
    def alu(self, name, rd, x, dly=0):
        i, v = _op2(x)
        return self.raw(word(OP[name], a=rd, i=i, imm=v, dly=dly))

    def mov(self, rd, x, dly=0):
        return self.alu("mov", rd, x, dly)

    def add(self, rd, x, dly=0):
        return self.alu("add", rd, x, dly)

    def sub(self, rd, x, dly=0):
        return self.alu("sub", rd, x, dly)

    def and_(self, rd, x, dly=0):
        return self.alu("and_", rd, x, dly)

    def or_(self, rd, x, dly=0):
        return self.alu("or_", rd, x, dly)

    def xor(self, rd, x, dly=0):
        return self.alu("xor", rd, x, dly)

    def cmp(self, rd, x, dly=0):
        return self.alu("cmp", rd, x, dly)

    def shl(self, rd, n, dly=0):
        return self.raw(word(OP["shl"], a=rd, i=1, imm=n & 0xF, dly=dly))

    def shr(self, rd, n, dly=0):
        return self.raw(word(OP["shr"], a=rd, i=1, imm=n & 0xF, dly=dly))

    def par(self, rd, dly=0):
        return self.raw(word(OP["par"], a=rd, dly=dly))

    # pins
    def set(self, pin, src, dly=0):
        return self.raw(word(OP["set"], imm=(pin << 12) | src, dly=dly))

    def setoe(self, pin, v, dly=0):
        return self.raw(word(OP["setoe"], imm=(pin << 12) | v, dly=dly))

    def wait(self, pin, lvl, rd=None, dly=0):
        return self.raw(word(OP["wait"], a=rd or 0, i=int(rd is not None),
                             imm=(pin << 12) | (lvl << 11), dly=dly))

    def jpin(self, pin, lvl, t, dly=0):
        return self._e(lambda lab: word(OP["jpin"], imm=(pin << 12) | (lvl << 11) | self._t(lab, t),
                                        dly=dly))

    def rdpins(self, rd, dly=0):
        return self.raw(word(OP["rdpins"], a=rd, dly=dly))

    def bitin(self, rd, pin, msb=1, dly=0):
        return self.raw(word(OP["bitin"], a=rd, imm=(pin << 12) | msb, dly=dly))

    def bitout(self, pin, rd, msb=1, dly=0):
        return self.raw(word(OP["bitout"], a=rd, imm=(pin << 12) | msb, dly=dly))

    # FIFOs / memory
    def fget(self, rd, fifo, nb=0, dly=0):
        return self.raw(word(OP["fget"], a=rd, b=fifo, imm=nb, dly=dly))

    def fput(self, rs, fifo, nb=0, dly=0):
        return self.raw(word(OP["fput"], a=rs, b=fifo, imm=nb, dly=dly))

    def ld(self, rd, ra, inc=0, dly=0):
        return self.raw(word(OP["ld"], a=rd, b=ra, imm=inc, dly=dly))

    def st(self, rs, ra, inc=0, dly=0):
        return self.raw(word(OP["st"], a=rs, b=ra, imm=inc, dly=dly))

    # units
    def ucfg(self, u, idx, x, dly=0):
        i, v = _op2(x)
        return self.raw(word(OP["ucfg"], a=u, i=i, b=idx, imm=v, dly=dly))

    def urd(self, rd, u, idx, dly=0):
        return self.raw(word(OP["urd"], a=rd, b=u, imm=idx, dly=dly))

    def ustart(self, mask, data=None, dly=0):
        if data is None:
            return self.raw(word(OP["ustart"], a=mask, dly=dly))
        i, v = _op2(data)
        return self.raw(word(OP["ustart"], a=mask, b=1, i=i, imm=v, dly=dly))

    def ustop(self, mask, dly=0):
        return self.raw(word(OP["ustop"], a=mask, dly=dly))

    def uwait(self, mask, dly=0):
        return self.raw(word(OP["uwait"], a=mask, dly=dly))

    def evt(self, d, sub, dly=0):
        return self.raw(word(OP["evt"], a=d, b=sub, dly=dly))

    def host_irq(self, dly=0):
        return self.raw(word(OP["host"], b=0, dly=dly))

    def dbclr(self, dly=0):
        return self.raw(word(OP["host"], b=1, dly=dly))
