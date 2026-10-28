/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_decode: instruction word -> one-hot op + fields (combinational).
 *
 * Word layout (IW >= 24; bits [IW-9:16] are reserved when IW > 24):
 *   [IW-1 -: 5] opcode   [IW-6 -: 3] delay (0-7 extra stall cycles)   [15:0] operands
 *
 * Operands (see docs/spec/isa.md):
 *   op  mnemonic                  [15:12] [11]  [10]   [9]    [8]   [7:0]
 *   0   nop
 *   1   set   pin, src            pin                                [1:0] src: 0,1,C,~C
 *   2   setoe pin, v              pin                                [0] v
 *   3   out   pin, n              pin     [11:8] n-1
 *   4   in    pin, n              pin     [11:8] n-1
 *   5   wait  pin, lvl [,to ctrX] pin     lvl   to_en  to_ctr
 *   6   jpin  pin, lvl, target    pin     lvl                       target
 *   7   jmp   [cond,] target                    [10:8] cond          target
 *   8   djnz  ctrX, target                                    ctr   target
 *   9   mov   ctrX, imm15         [15] ctr, [14:0] imm
 *   10  delay n                   [15:0] n
 *   11  pull  [nb]                                                   [0] nb
 *   12  push  [nb]                                                   [0] nb
 *   13  vote  pin                 pin
 *   14-31 reserved (executed as nop, op_illegal = 1)
 *
 * jmp cond: 0 always, 1 C, 2 !C, 3 TO, 4 !TO, 5 TX FIFO empty, 6 RX FIFO full,
 *           7 OSR empty (osr_cnt == 0)
 */

`default_nettype none

module pe_decode #(
    parameter IW = 24,
    parameter AW = 6
) (
    input  wire [IW-1:0] ir,
    // one-hot op
    output wire          op_set,
    output wire          op_setoe,
    output wire          op_out,
    output wire          op_in,
    output wire          op_wait,
    output wire          op_jpin,
    output wire          op_jmp,
    output wire          op_djnz,
    output wire          op_mov,
    output wire          op_delay,
    output wire          op_pull,
    output wire          op_push,
    output wire          op_vote,
    output wire          op_illegal,
    // fields
    output wire [   2:0] dly,
    output wire [   3:0] pin,
    output wire [   4:0] n,
    output wire          lvl,
    output wire          to_en,
    output wire          to_ctr,
    output wire [   2:0] cond,
    output wire          ctr_sel,
    output wire [   1:0] src,
    output wire          nb,
    output wire [AW-1:0] target,
    output wire [  15:0] imm
);

  localparam [4:0] OP_SET = 5'd1;
  localparam [4:0] OP_SETOE = 5'd2;
  localparam [4:0] OP_OUT = 5'd3;
  localparam [4:0] OP_IN = 5'd4;
  localparam [4:0] OP_WAIT = 5'd5;
  localparam [4:0] OP_JPIN = 5'd6;
  localparam [4:0] OP_JMP = 5'd7;
  localparam [4:0] OP_DJNZ = 5'd8;
  localparam [4:0] OP_MOV = 5'd9;
  localparam [4:0] OP_DELAY = 5'd10;
  localparam [4:0] OP_PULL = 5'd11;
  localparam [4:0] OP_PUSH = 5'd12;
  localparam [4:0] OP_VOTE = 5'd13;

  wire [4:0] opc = ir[IW-1-:5];

  assign op_set     = (opc == OP_SET);
  assign op_setoe   = (opc == OP_SETOE);
  assign op_out     = (opc == OP_OUT);
  assign op_in      = (opc == OP_IN);
  assign op_wait    = (opc == OP_WAIT);
  assign op_jpin    = (opc == OP_JPIN);
  assign op_jmp     = (opc == OP_JMP);
  assign op_djnz    = (opc == OP_DJNZ);
  assign op_mov     = (opc == OP_MOV);
  assign op_delay   = (opc == OP_DELAY);
  assign op_pull    = (opc == OP_PULL);
  assign op_push    = (opc == OP_PUSH);
  assign op_vote    = (opc == OP_VOTE);
  assign op_illegal = (opc > OP_VOTE);

  assign dly        = ir[IW-6-:3];
  assign pin        = ir[15:12];
  assign n          = {1'b0, ir[11:8]} + 5'd1;
  assign lvl        = ir[11];
  assign to_en      = ir[10];
  assign to_ctr     = ir[9];
  assign cond       = ir[10:8];
  assign ctr_sel    = op_mov ? ir[15] : ir[8];
  assign src        = ir[1:0];
  assign nb         = ir[0];
  assign target     = ir[AW-1:0];
  assign imm        = op_mov ? {1'b0, ir[14:0]} : ir[15:0];

  generate
    if (IW > 24) begin : g_unused
      wire _unused = &{1'b0, ir[IW-9:16]};
    end
  endgenerate

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // At most one op flag is set; reserved opcodes (14-31) set only
  // op_illegal; opcode 0 (nop) sets nothing; n is always 1..16; fields
  // come from the documented bit positions.
  wire [13:0] f_ops = {op_set, op_setoe, op_out, op_in, op_wait, op_jpin, op_jmp, op_djnz,
                       op_mov, op_delay, op_pull, op_push, op_vote, op_illegal};
  always @(*) begin
    assert ($onehot0(f_ops));
    assert (op_illegal == (opc > 5'd13));
    if (opc == 5'd0) assert (f_ops == 14'd0);
    if (opc != 5'd0) assert (f_ops != 14'd0);
    assert (n >= 5'd1 && n <= 5'd16);
    assert (dly == ir[IW-6-:3]);
    assert (target == ir[AW-1:0]);
    assert (imm == (op_mov ? {1'b0, ir[14:0]} : ir[15:0]));
  end
`endif

endmodule
