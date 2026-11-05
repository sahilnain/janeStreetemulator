# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_decode: unit test for src/core/pe_decode.v (32-bit word).

What is checked
---------------
1. all_opcodes: every one of the 64 opcodes with random other fields; each
   field is the documented slice, and `illegal` is set exactly for the
   reserved opcodes (5, 35-63).
2. random_words: 5000 random words, all fields against a Python decode.
3. field_extremes: all-zero, all-one and walking-one words.
"""

import random

import cocotb
import pe_isa as I
from tbutil import param, settle, v

AW = param("AW", 6)


def ref(w):
    return dict(opcode=w >> 26, dly=(w >> 23) & 7, fa=(w >> 20) & 7, fi=(w >> 19) & 1,
                fb=(w >> 16) & 7, imm=w & 0xFFFF, cond=(w >> 11) & 0x1F,
                target=w & ((1 << AW) - 1), pin=(w >> 12) & 0xF, lvl=(w >> 11) & 1, rs=w & 7,
                illegal=int((w >> 26) in I.RESERVED_OPS))


async def check(dut, w):
    dut.ir.value = w
    await settle()
    for k, e in ref(w).items():
        assert v(getattr(dut, k)) == e, f"{k} of {w:08x}: {v(getattr(dut, k))} != {e}"


@cocotb.test()
async def all_opcodes(dut):
    for op in range(64):
        for _ in range(8):
            await check(dut, (op << 26) | random.getrandbits(26))


@cocotb.test()
async def random_words(dut):
    for _ in range(5000):
        await check(dut, random.getrandbits(32))


@cocotb.test()
async def field_extremes(dut):
    for w in [0, 0xFFFFFFFF] + [1 << i for i in range(32)] + [~(1 << i) & 0xFFFFFFFF for i in range(32)]:
        await check(dut, w)
