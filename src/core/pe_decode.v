/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_decode: 32-bit instruction word -> opcode + fields (combinational).
 *
 * Word layout (docs/spec/isa.md Section 2):
 *   [31:26] opcode  [25:23] dly  [22:20] A  [19] I  [18:16] B  [15:0] imm16
 *
 * Derived fields:
 *   cond   = imm[15:11]   jump condition
 *   target = imm[9:0]     jump target (low AW bits used)
 *   pin    = imm[15:12]   pin number (pin instructions)
 *   lvl    = imm[11]      level (wait, jpin)
 *   rs     = imm[2:0]     register operand when I = 0
 *
 * `illegal` is set for the reserved opcodes (5, 35-63); they execute as nop
 * and set STATUS2.ill_err.
 */

`default_nettype none

module pe_decode #(
    parameter AW = 6
) (
    input  wire [  31:0] ir,
    output wire [   5:0] opcode,
    output wire [   2:0] dly,
    output wire [   2:0] fa,
    output wire          fi,
    output wire [   2:0] fb,
    output wire [  15:0] imm,
    output wire [   4:0] cond,
    output wire [AW-1:0] target,
    output wire [   3:0] pin,
    output wire          lvl,
    output wire [   2:0] rs,
    output wire          illegal
);

  assign opcode = ir[31:26];
  assign dly    = ir[25:23];
  assign fa     = ir[22:20];
  assign fi     = ir[19];
  assign fb     = ir[18:16];
  assign imm    = ir[15:0];
  assign cond   = ir[15:11];
  assign pin    = ir[15:12];
  assign lvl    = ir[11];
  assign rs     = ir[2:0];

  // target is imm[9:0]; AW <= 10
  wire [9:0] tgt10 = ir[9:0];
  assign target = tgt10[AW-1:0];

  assign illegal = (opcode == 6'd5) || (opcode > 6'd34);

  wire _unused = &{1'b0, tgt10};

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // Pure wiring: every field is the documented slice of ir, and exactly the
  // reserved opcodes are illegal.
  always @(*) begin
    assert ({opcode, dly, fa, fi, fb, imm} == ir);
    assert (cond == imm[15:11] && pin == imm[15:12] && lvl == imm[11] && rs == imm[2:0]);
    assert (target == ir[AW-1:0]);
    assert (illegal == !(opcode <= 6'd4 || (opcode >= 6'd6 && opcode <= 6'd34)));
  end
`endif

endmodule
