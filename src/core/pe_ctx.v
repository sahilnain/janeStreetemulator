/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_ctx: architectural state of one execution context.
 *
 *   pc, ctr0, ctr1, osr, osr_cnt (bits left in OSR), isr, flags C and TO,
 *   stall counter.
 *
 * - The stall counter counts down every clock cycle, independent of slots.
 * - ready = run && stall == 0 && !vote_busy. pe_core only issues a ready
 *   context in its slot.
 * - `we` writes back the next state computed by pe_exec (in this
 *   context's slot).
 * - `vote_done` shifts the vote result into ISR and copies it to C. It never
 *   coincides with `we`, because vote_busy covers the done cycle.
 * - `restart` loads start_pc, clears state and sets stall = 1. The stall
 *   covers the instruction already fetched from the old PC, so the first
 *   instruction issued comes from start_pc.
 */

`default_nettype none

module pe_ctx #(
    parameter AW      = 6,
    parameter SR_W    = 16,
    parameter CTR_W   = 16,
    parameter STALL_W = 17,
    // derived, do not override
    parameter CNT_W   = $clog2(SR_W + 1)
) (
    input  wire               clk,
    input  wire               rst,
    // control from pe_regs
    input  wire               run,
    input  wire               restart,
    input  wire [     AW-1:0] start_pc,
    input  wire               shift_msb,
    // writeback from pe_exec
    input  wire               we,
    input  wire [     AW-1:0] pc_n,
    input  wire [  CTR_W-1:0] ctr0_n,
    input  wire [  CTR_W-1:0] ctr1_n,
    input  wire [   SR_W-1:0] osr_n,
    input  wire [  CNT_W-1:0] osr_cnt_n,
    input  wire [   SR_W-1:0] isr_n,
    input  wire               c_n,
    input  wire               to_n,
    input  wire [STALL_W-1:0] stall_n,
    // vote unit
    input  wire               vote_busy,
    input  wire               vote_done,
    input  wire               vote_bit,
    // state
    output reg  [     AW-1:0] pc,
    output reg  [  CTR_W-1:0] ctr0,
    output reg  [  CTR_W-1:0] ctr1,
    output reg  [   SR_W-1:0] osr,
    output reg  [  CNT_W-1:0] osr_cnt,
    output reg  [   SR_W-1:0] isr,
    output reg                c,
    output reg                to,
    output wire               ready
);

  reg [STALL_W-1:0] stall;

  assign ready = run & (stall == {STALL_W{1'b0}}) & ~vote_busy;

  wire [SR_W-1:0] isr_vote = shift_msb ? {isr[SR_W-2:0], vote_bit} : {vote_bit, isr[SR_W-1:1]};

  always @(posedge clk) begin
    if (rst) begin
      pc      <= {AW{1'b0}};
      ctr0    <= {CTR_W{1'b0}};
      ctr1    <= {CTR_W{1'b0}};
      osr     <= {SR_W{1'b0}};
      osr_cnt <= {CNT_W{1'b0}};
      isr     <= {SR_W{1'b0}};
      c       <= 1'b0;
      to      <= 1'b0;
      stall   <= {STALL_W{1'b0}};
    end else if (restart) begin
      pc      <= start_pc;
      ctr0    <= {CTR_W{1'b0}};
      ctr1    <= {CTR_W{1'b0}};
      osr     <= {SR_W{1'b0}};
      osr_cnt <= {CNT_W{1'b0}};
      isr     <= {SR_W{1'b0}};
      c       <= 1'b0;
      to      <= 1'b0;
      stall   <= {{(STALL_W - 1) {1'b0}}, 1'b1};
    end else begin
      if (we) begin
        pc      <= pc_n;
        ctr0    <= ctr0_n;
        ctr1    <= ctr1_n;
        osr     <= osr_n;
        osr_cnt <= osr_cnt_n;
        isr     <= isr_n;
        c       <= c_n;
        to      <= to_n;
        stall   <= stall_n;
      end else if (stall != {STALL_W{1'b0}}) begin
        stall <= stall - 1'b1;
      end
      if (vote_done) begin
        isr <= isr_vote;
        c   <= vote_bit;
      end
    end
  end

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // Contract with pe_core (assumed when pe_ctx is proved alone, ASSERTED
  // when proved inside pe_core): writeback only happens when the context
  // is ready, and vote_done only while the vote unit is busy.
  // Properties: ready = run & stall==0 & !vote_busy; restart loads
  // start_pc, clears state and sets stall = 1; writeback stores exactly
  // the next state; otherwise state holds, the stall counts down by 1
  // (saturating at 0), and vote_done shifts the vote bit into ISR and C.
`ifdef PE_CTX_TOP
  `define PE_CTX_ASSUME assume
  initial assume (rst);
`else
  `define PE_CTX_ASSUME assert
`endif
  reg f_rst_seen = 1'b0;
  always @(posedge clk) if (rst) f_rst_seen <= 1'b1;
  wire f_ok = f_rst_seen & ~rst;
  reg  f_past_ok = 1'b0;
  always @(posedge clk) f_past_ok <= f_ok;

  always @(*) begin
    if (f_ok) begin
      if (we) `PE_CTX_ASSUME (ready);
      if (vote_done) `PE_CTX_ASSUME (vote_busy);
      assert (ready == (run && stall == {STALL_W{1'b0}} && !vote_busy));
    end
  end

  always @(posedge clk) begin
    if (f_past_ok && f_ok) begin
      if ($past(restart)) begin
        assert (pc == $past(start_pc) && stall == 1);
        assert (ctr0 == 0 && ctr1 == 0 && osr == 0 && osr_cnt == 0 && isr == 0 && !c && !to);
      end else if ($past(we)) begin
        assert (pc == $past(pc_n) && ctr0 == $past(ctr0_n) && ctr1 == $past(ctr1_n));
        assert (osr == $past(osr_n) && osr_cnt == $past(osr_cnt_n) && isr == $past(isr_n));
        assert (c == $past(c_n) && to == $past(to_n) && stall == $past(stall_n));
      end else begin
        assert (pc == $past(pc) && ctr0 == $past(ctr0) && ctr1 == $past(ctr1));
        assert (osr == $past(osr) && osr_cnt == $past(osr_cnt) && to == $past(to));
        assert (stall == (($past(stall) == 0) ? 0 : $past(stall) - 1'b1));
        if ($past(vote_done)) assert (isr == $past(isr_vote) && c == $past(vote_bit));
        else assert (isr == $past(isr) && c == $past(c));
      end
    end
  end
`undef PE_CTX_ASSUME
`endif

endmodule
