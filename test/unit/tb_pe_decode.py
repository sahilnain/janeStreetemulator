# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_decode: unit test for src/core/pe_decode.v.

What is checked
---------------
The RTL decoder is compared with the reference decoder `pe_isa.decode()`,
which is the same encoding the test encoders use.
1. every_opcode: all 32 opcodes x 64 random operand/delay patterns:
   one-hot op flags, op_illegal for 14-31, and every field.
2. encoder_roundtrip: words built by the encoder functions (set_, out,
   jmp, ...) decode back to the intended fields.
"""

import random

import cocotb
import pe_isa as I
from tbutil import param, settle, v

FIELDS = ["op_set", "op_setoe", "op_out", "op_in", "op_wait", "op_jpin", "op_jmp", "op_djnz",
          "op_mov", "op_delay", "op_pull", "op_push", "op_vote", "op_illegal", "dly", "pin", "n",
          "lvl", "to_en", "to_ctr", "cond", "ctr_sel", "src", "nb", "target", "imm"]


async def check_word(dut, w, iw, aw):
    dut.ir.value = w
    await settle()
    ref = I.decode(w, iw, aw)
    for f in FIELDS:
        assert v(getattr(dut, f)) == ref[f], f"word {w:0{iw // 4}x}: {f} = {v(getattr(dut, f))}, expected {ref[f]}"
    assert sum(v(getattr(dut, f)) for f in FIELDS[:13]) <= 1, "op flags not one-hot"


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def every_opcode(dut):
    iw, aw = param("IW", 24), param("AW", 6)
    for opc in range(32):
        for _ in range(64):
            w = (opc << (iw - 5)) | random.getrandbits(iw - 5)
            await check_word(dut, w, iw, aw)


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def encoder_roundtrip(dut):
    iw, aw = param("IW", 24), param("AW", 6)
    if iw != I.IW_DEFAULT:
        return  # the encoders target IW = 24
    cases = [
        (I.set_(5, I.SRC_C, dly=3), dict(op_set=1, pin=5, src=2, dly=3)),
        (I.setoe(2, 1), dict(op_setoe=1, pin=2, src=1)),
        (I.out(7, 8), dict(op_out=1, pin=7, n=8)),
        (I.in_(1, 16), dict(op_in=1, pin=1, n=16)),
        (I.wait(3, 1, to_ctr=1), dict(op_wait=1, pin=3, lvl=1, to_en=1, to_ctr=1)),
        (I.wait(3, 0), dict(op_wait=1, pin=3, lvl=0, to_en=0)),
        (I.jpin(4, 1, 33), dict(op_jpin=1, pin=4, lvl=1, target=33 & ((1 << aw) - 1))),
        (I.jmp(17, I.OSR_EMPTY), dict(op_jmp=1, cond=7, target=17)),
        (I.djnz(1, 9), dict(op_djnz=1, ctr_sel=1, target=9)),
        (I.mov(1, 0x7ABC), dict(op_mov=1, ctr_sel=1, imm=0x7ABC)),
        (I.mov(0, 5), dict(op_mov=1, ctr_sel=0, imm=5)),
        (I.delay(0xFFFF, dly=7), dict(op_delay=1, imm=0xFFFF, dly=7)),
        (I.pull(1), dict(op_pull=1, nb=1)),
        (I.push(0), dict(op_push=1, nb=0)),
        (I.vote(6), dict(op_vote=1, pin=6)),
        (I.nop(), dict(op_illegal=0)),
    ]
    for w, exp in cases:
        dut.ir.value = w
        await settle()
        for f, x in exp.items():
            assert v(getattr(dut, f)) == x, f"{w:06x}: {f}"
