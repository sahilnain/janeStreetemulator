/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_exec: shared datapath (combinational). All contexts share it.
 *
 * Given the decoded instruction and the state of the context that owns the
 * current slot, produce that context's next state and the side-effect
 * requests (pins, FIFOs, vote). pe_core gates every side effect with `issue`.
 *
 * An instruction either completes (done = 1: PC advances or jumps, stall is
 * loaded from the delay field, plus n for `delay`), or is held (done = 0: PC
 * and stall are unchanged and it is retried in the next slot). Held cases:
 *   wait (level not met), blocking pull on an empty TX FIFO,
 *   blocking push on a full RX FIFO.
 * A held `wait` with a timeout still decrements its counter. When the
 * counter reaches 0, the wait completes with TO = 1.
 *
 * Shift direction (per context, shift_msb):
 *   LSB-first: pull loads OSR[DATA_W-1:0]; out takes the low n bits and
 *              shifts right. in shifts right, so new bits enter at the top;
 *              push sends ISR[SR_W-1 -: DATA_W].
 *   MSB-first: pull loads OSR[SR_W-1 -: DATA_W]; out takes the top n bits
 *              and shifts left. in shifts left; push sends ISR[DATA_W-1:0].
 * For out/in, bit 0 of the n-bit value maps to pin `pin`, bit 1 to pin+1, ...
 */

`default_nettype none

module pe_exec #(
    parameter AW       = 6,
    parameter NUM_PINS = 8,
    parameter SR_W     = 16,
    parameter DATA_W   = 8,
    parameter CTR_W    = 16,
    parameter STALL_W  = 17,
    // derived, do not override
    parameter CNT_W    = $clog2(SR_W + 1)
) (
    // decoded instruction
    input wire          op_set,
    input wire          op_setoe,
    input wire          op_out,
    input wire          op_in,
    input wire          op_wait,
    input wire          op_jpin,
    input wire          op_jmp,
    input wire          op_djnz,
    input wire          op_mov,
    input wire          op_delay,
    input wire          op_pull,
    input wire          op_push,
    input wire          op_vote,
    input wire [   2:0] dly,
    input wire [   3:0] pin,
    input wire [   4:0] n,
    input wire          lvl,
    input wire          to_en,
    input wire          to_ctr,
    input wire [   2:0] cond,
    input wire          ctr_sel,
    input wire [   1:0] src,
    input wire          nb,
    input wire [AW-1:0] target,
    input wire [  15:0] imm,

    // current context state
    input wire [   AW-1:0] pc,
    input wire [CTR_W-1:0] ctr0,
    input wire [CTR_W-1:0] ctr1,
    input wire [ SR_W-1:0] osr,
    input wire [CNT_W-1:0] osr_cnt,
    input wire [ SR_W-1:0] isr,
    input wire             c,
    input wire             to,
    input wire             shift_msb,
    input wire             vote_avail,

    // environment
    input wire [NUM_PINS-1:0] pin_in,
    input wire [  DATA_W-1:0] txf_rdata,
    input wire                txf_empty,
    input wire                rxf_full,

    // next state
    output reg               done,
    output reg [     AW-1:0] pc_n,
    output reg [  CTR_W-1:0] ctr0_n,
    output reg [  CTR_W-1:0] ctr1_n,
    output reg [   SR_W-1:0] osr_n,
    output reg [  CNT_W-1:0] osr_cnt_n,
    output reg [   SR_W-1:0] isr_n,
    output reg               c_n,
    output reg               to_n,
    output reg [STALL_W-1:0] stall_n,

    // side effects (gated by `issue` in pe_core)
    output reg                pin_wr_en,
    output reg [NUM_PINS-1:0] pin_wr_mask,
    output reg [NUM_PINS-1:0] pin_wr_val,
    output reg                pin_oe_en,
    output reg [NUM_PINS-1:0] pin_oe_mask,
    output reg [NUM_PINS-1:0] pin_oe_val,
    output reg                txf_rd,
    output reg                rxf_wr,
    output reg [  DATA_W-1:0] rxf_wdata,
    output reg                vote_start,
    output reg [         3:0] vote_pin
);

  // Wide scratch width for pin/shift arithmetic, so that no shift can lose bits.
  localparam WX = SR_W + NUM_PINS + 16;
  localparam integer SR_W_I = SR_W;
  localparam [4:0] SR_W5 = SR_W_I[4:0];

  // n clamped to SR_W
  wire [4:0] n_eff = (n > SR_W5) ? SR_W5 : n;
  wire [SR_W-1:0] nmask = ~({SR_W{1'b1}} << n_eff);

  // ---- out: OSR -> pins ----
  wire [SR_W-1:0] out_bits = shift_msb ? (osr >> (SR_W5 - n_eff)) : (osr & nmask);
  wire [SR_W-1:0] osr_out = shift_msb ? (osr << n_eff) : (osr >> n_eff);
  wire [CNT_W-1:0] n_cnt = n_eff[CNT_W-1:0];
  wire [CNT_W-1:0] osr_cnt_out = (osr_cnt > n_cnt) ? (osr_cnt - n_cnt) : {CNT_W{1'b0}};
  wire [WX-1:0] out_mask_x = {{(WX - SR_W) {1'b0}}, nmask} << pin;
  wire [WX-1:0] out_val_x = {{(WX - SR_W) {1'b0}}, out_bits} << pin;

  // ---- pins -> in / wait / jpin ----
  wire [WX-1:0] pin_x = {{(WX - NUM_PINS) {1'b0}}, pin_in} >> pin;
  wire pin_bit = pin_x[0];
  wire [SR_W-1:0] in_bits = pin_x[SR_W-1:0] & nmask;
  wire [SR_W-1:0] isr_in = shift_msb ? ((isr << n_eff) | in_bits)
                                     : ((isr >> n_eff) | (in_bits << (SR_W5 - n_eff)));

  // ---- single-pin mask for set / setoe ----
  wire [WX-1:0] one_x = {{(WX - 1) {1'b0}}, 1'b1} << pin;
  wire [NUM_PINS-1:0] one_mask = one_x[NUM_PINS-1:0];
  wire set_bit = src[1] ? (c ^ src[0]) : src[0];

  // ---- FIFO data alignment ----
  wire [SR_W-1:0] pull_lsb = {{(SR_W - DATA_W) {1'b0}}, txf_rdata};
  wire [SR_W-1:0] pull_val = shift_msb ? (pull_lsb << (SR_W - DATA_W)) : pull_lsb;
  wire [SR_W-1:0] isr_top = isr >> (SR_W - DATA_W);
  wire [DATA_W-1:0] push_val = shift_msb ? isr[DATA_W-1:0] : isr_top[DATA_W-1:0];

  // ---- counters ----
  wire [CTR_W-1:0] ctr_s = ctr_sel ? ctr1 : ctr0;
  wire [CTR_W-1:0] ctr_s_dec = ctr_s - 1'b1;
  wire [CTR_W-1:0] ctr_t = to_ctr ? ctr1 : ctr0;
  wire [CTR_W-1:0] ctr_imm = imm[CTR_W-1:0];

  // ---- jmp condition ----
  reg cond_ok;
  always @(*) begin
    case (cond)
      3'd0: cond_ok = 1'b1;
      3'd1: cond_ok = c;
      3'd2: cond_ok = ~c;
      3'd3: cond_ok = to;
      3'd4: cond_ok = ~to;
      3'd5: cond_ok = txf_empty;
      3'd6: cond_ok = rxf_full;
      default: cond_ok = (osr_cnt == {CNT_W{1'b0}});
    endcase
  end

  reg jump;

  // Only the low bits of the wide scratch signals are used.
  wire _unused = &{1'b0, out_mask_x[WX-1:NUM_PINS], out_val_x[WX-1:NUM_PINS], pin_x[WX-1:SR_W],
                   one_x[WX-1:NUM_PINS], isr_top[SR_W-1:DATA_W]};

  always @(*) begin
    // defaults: nothing changes, instruction completes
    done        = 1'b1;
    jump        = 1'b0;
    ctr0_n      = ctr0;
    ctr1_n      = ctr1;
    osr_n       = osr;
    osr_cnt_n   = osr_cnt;
    isr_n       = isr;
    c_n         = c;
    to_n        = to;
    pin_wr_en   = 1'b0;
    pin_wr_mask = {NUM_PINS{1'b0}};
    pin_wr_val  = {NUM_PINS{1'b0}};
    pin_oe_en   = 1'b0;
    pin_oe_mask = {NUM_PINS{1'b0}};
    pin_oe_val  = {NUM_PINS{1'b0}};
    txf_rd      = 1'b0;
    rxf_wr      = 1'b0;
    rxf_wdata   = push_val;
    vote_start  = 1'b0;
    vote_pin    = pin;

    if (op_set) begin
      pin_wr_en   = 1'b1;
      pin_wr_mask = one_mask;
      pin_wr_val  = {NUM_PINS{set_bit}};
    end

    if (op_setoe) begin
      pin_oe_en   = 1'b1;
      pin_oe_mask = one_mask;
      pin_oe_val  = {NUM_PINS{src[0]}};
    end

    if (op_out) begin
      pin_wr_en   = 1'b1;
      pin_wr_mask = out_mask_x[NUM_PINS-1:0];
      pin_wr_val  = out_val_x[NUM_PINS-1:0];
      osr_n       = osr_out;
      osr_cnt_n   = osr_cnt_out;
    end

    if (op_in) begin
      isr_n = isr_in;
    end

    if (op_wait) begin
      if (pin_bit == lvl) begin
        if (to_en) to_n = 1'b0;
      end else if (to_en && ctr_t == {CTR_W{1'b0}}) begin
        to_n = 1'b1;
      end else begin
        done = 1'b0;
        if (to_en) begin
          if (to_ctr) ctr1_n = ctr1 - 1'b1;
          else ctr0_n = ctr0 - 1'b1;
        end
      end
    end

    if (op_jpin) jump = (pin_bit == lvl);

    if (op_jmp) jump = cond_ok;

    if (op_djnz) begin
      if (ctr_sel) ctr1_n = ctr_s_dec;
      else ctr0_n = ctr_s_dec;
      jump = (ctr_s_dec != {CTR_W{1'b0}});
    end

    if (op_mov) begin
      if (ctr_sel) ctr1_n = ctr_imm;
      else ctr0_n = ctr_imm;
    end

    if (op_pull) begin
      if (!txf_empty) begin
        txf_rd    = 1'b1;
        osr_n     = pull_val;
        osr_cnt_n = DATA_W[CNT_W-1:0];
      end else if (!nb) begin
        done = 1'b0;
      end
    end

    if (op_push) begin
      if (rxf_full && !nb) begin
        done = 1'b0;
      end else begin
        rxf_wr = 1'b1;  // full + nb: pe_fifo drops it and flags overrun
        isr_n  = {SR_W{1'b0}};
      end
    end

    if (op_vote && vote_avail) vote_start = 1'b1;

    // completion
    if (done) begin
      pc_n    = jump ? target : pc + 1'b1;
      stall_n = {{(STALL_W - 3) {1'b0}}, dly} + (op_delay ? {{(STALL_W - 16) {1'b0}}, imm} : {STALL_W{1'b0}});
    end else begin
      pc_n    = pc;
      stall_n = {STALL_W{1'b0}};
    end
  end

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // Combinational, so every property is proved for ALL instructions, states
  // and environments. Contract (assumed alone, proved inside pe_core via
  // pe_decode): at most one op flag, n in 1..16.
  // * Held instructions (done = 0) are only wait / blocking pull / blocking
  //   push, keep PC, load no stall, and have NO side effects; only a
  //   timed-out wait's counter may change.
  // * Side effects only from their own op: FIFO read only from a non-empty
  //   TX FIFO, RX write only if not full or nb, pins only via set/out/setoe,
  //   vote only if the context has a vote unit.
  // * PC is pc+1 except for jumps; stall = dly (+ n for `delay`); C is never
  //   changed by exec.
  // * `out` loses no bits: OSR' and the bits sent to the pins recombine to
  //   OSR (both shift directions); out writes no pin outside its mask.
  // * `in`: the new bits land in the right end of ISR and the old bits move
  //   over unchanged.
`ifdef PE_EXEC_TOP
  `define PE_EXEC_ASSUME assume
`else
  `define PE_EXEC_ASSUME assert
`endif
  wire [12:0] f_ops = {op_set, op_setoe, op_out, op_in, op_wait, op_jpin, op_jmp, op_djnz,
                       op_mov, op_delay, op_pull, op_push, op_vote};

  always @(*) begin
    `PE_EXEC_ASSUME ($onehot0(f_ops));
    `PE_EXEC_ASSUME (n >= 5'd1 && n <= 5'd16);

    if (!done) begin
      assert (op_wait || op_pull || op_push);
      assert (pc_n == pc && stall_n == {STALL_W{1'b0}});
      assert (!pin_wr_en && !pin_oe_en && !txf_rd && !rxf_wr && !vote_start);
      assert (osr_n == osr && osr_cnt_n == osr_cnt && isr_n == isr && c_n == c && to_n == to);
      if (!(op_wait && to_en)) assert (ctr0_n == ctr0 && ctr1_n == ctr1);
    end
    if (op_pull && !txf_empty) assert (done);
    if (op_push && (!rxf_full || nb)) assert (done);

    if (txf_rd) assert (op_pull && !txf_empty);
    if (rxf_wr) assert (op_push && (!rxf_full || nb));
    if (pin_wr_en) assert (op_set || op_out);
    if (pin_oe_en) assert (op_setoe);
    if (vote_start) assert (op_vote && vote_avail);
    if (op_set) assert ($onehot0(pin_wr_mask));

    if (done && !(op_jmp || op_jpin || op_djnz)) assert (pc_n == pc + 1'b1);
    if (done) assert (stall_n == dly + (op_delay ? imm : 16'd0));
    assert (c_n == c);

    if (op_out) begin
      assert ((pin_wr_val & ~pin_wr_mask) == {NUM_PINS{1'b0}});
      if (!shift_msb) assert (((osr_n << n_eff) | out_bits) == osr);
      else assert (((osr_n >> n_eff) | (out_bits << (SR_W5 - n_eff))) == osr);
    end
    if (op_in) begin
      if (!shift_msb) begin
        assert ((isr_n << n_eff) == ((isr >> n_eff) << n_eff));
        assert ((isr_n >> (SR_W5 - n_eff)) == in_bits);
      end else begin
        assert ((isr_n >> n_eff) == ((isr << n_eff) >> n_eff));
        assert ((isr_n & nmask) == in_bits);
      end
    end
    if (txf_rd) assert (osr_cnt_n == DATA_W[CNT_W-1:0]);
    if (rxf_wr) assert (isr_n == {SR_W{1'b0}});
  end
`undef PE_EXEC_ASSUME
`endif

endmodule
