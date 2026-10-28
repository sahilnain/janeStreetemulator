/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_fifo: synchronous FIFO, first-word fall-through (rdata is valid whenever !empty).
 *
 * - rd on empty is ignored.
 * - wr on full is dropped and sets the sticky `overrun` flag, unless a rd happens
 *   in the same cycle. Blocking is the core's job: it holds the instruction
 *   instead of asserting wr/rd.
 * - DEPTH does not need to be a power of two.
 */

`default_nettype none

module pe_fifo #(
    parameter W     = 8,
    parameter DEPTH = 4,
    // derived, do not override
    parameter CW    = $clog2(DEPTH + 1)
) (
    input  wire          clk,
    input  wire          rst,
    // write side
    input  wire          wr,
    input  wire [ W-1:0] wdata,
    output wire          full,
    // read side
    input  wire          rd,
    output wire [ W-1:0] rdata,
    output wire          empty,
    // status
    output wire [CW-1:0] count,
    output reg           overrun,
    input  wire          clr_overrun
);

  localparam PW = (DEPTH > 1) ? $clog2(DEPTH) : 1;
  localparam integer LAST_I = DEPTH - 1;
  localparam [PW-1:0] LAST = LAST_I[PW-1:0];

  reg [W-1:0] mem[0:DEPTH-1];
  reg [PW-1:0] wptr, rptr;
  reg [CW-1:0] cnt;

  assign empty = (cnt == {CW{1'b0}});
  assign full  = (cnt == DEPTH[CW-1:0]);
  assign count = cnt;
  assign rdata = mem[rptr];

  wire rd_ok = rd & ~empty;
  wire wr_ok = wr & (~full | rd_ok);

  always @(posedge clk) begin
    if (wr_ok) mem[wptr] <= wdata;
  end

  always @(posedge clk) begin
    if (rst) begin
      wptr    <= {PW{1'b0}};
      rptr    <= {PW{1'b0}};
      cnt     <= {CW{1'b0}};
      overrun <= 1'b0;
    end else begin
      if (wr_ok) wptr <= (wptr == LAST) ? {PW{1'b0}} : wptr + 1'b1;
      if (rd_ok) rptr <= (rptr == LAST) ? {PW{1'b0}} : rptr + 1'b1;
      if (wr_ok & ~rd_ok) cnt <= cnt + 1'b1;
      else if (rd_ok & ~wr_ok) cnt <= cnt - 1'b1;
      if (wr & ~wr_ok) overrun <= 1'b1;
      else if (clr_overrun) overrun <= 1'b0;
    end
  end

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // Invariants: count in range, pointers consistent with the count.
  // Transitions: count follows wr_ok/rd_ok, overrun is sticky (set wins).
  // Data integrity: one write, chosen by the solver (f_pick), is tracked
  // and must come out of rdata unchanged, after exactly the entries ahead
  // of it. Since any write can be the tracked one, this proves FIFO order
  // and no corruption for all data.
`ifdef PE_FIFO_TOP
  initial assume (rst);
`endif
  reg f_rst_seen = 1'b0;
  always @(posedge clk) if (rst) f_rst_seen <= 1'b1;
  wire f_ok = f_rst_seen & ~rst;
  reg  f_past_ok = 1'b0;
  always @(posedge clk) f_past_ok <= f_ok;

  wire [PW+CW:0] f_sum = rptr + cnt;
  wire [PW+CW:0] f_wexp = (f_sum >= DEPTH) ? f_sum - DEPTH : f_sum;

  always @(*) begin
    if (f_ok) begin
      assert (cnt <= DEPTH);
      assert (rptr <= LAST && wptr <= LAST);
      assert (wptr == f_wexp[PW-1:0]);
    end
  end

  always @(posedge clk) begin
    if (f_past_ok && f_ok) begin
      assert (cnt == $past(cnt) + $past(wr_ok) - $past(rd_ok));
      if ($past(wr & ~wr_ok)) assert (overrun);
      else if ($past(clr_overrun)) assert (!overrun);
      else assert (overrun == $past(overrun));
    end
  end

  (* anyseq *) wire f_pick;
  reg          f_trk;
  reg [ W-1:0] f_data;
  reg [CW-1:0] f_pos;  // entries ahead of the tracked one

  always @(posedge clk) begin
    if (rst) f_trk <= 1'b0;
    else if (!f_trk) begin
      if (wr_ok && f_pick) begin
        f_trk  <= 1'b1;
        f_data <= wdata;
        f_pos  <= cnt - rd_ok;
      end
    end else if (rd_ok) begin
      if (f_pos == {CW{1'b0}}) f_trk <= 1'b0;
      else f_pos <= f_pos - 1'b1;
    end
  end

  wire [PW+CW:0] f_isum = rptr + f_pos;
  wire [PW+CW:0] f_idx = (f_isum >= DEPTH) ? f_isum - DEPTH : f_isum;

  always @(*) begin
    if (f_ok && f_trk) begin
      assert (f_pos < cnt);
      assert (mem[f_idx[PW-1:0]] == f_data);
      if (f_pos == {CW{1'b0}}) assert (rdata == f_data);
    end
`ifdef PE_FIFO_TOP  // module-level covers only when this module is the proof top
    if (f_ok) begin
      cover (full);
      cover (overrun);
      cover (f_trk && rd_ok && f_pos == {CW{1'b0}} && cnt == DEPTH);
    end
`endif
  end
`endif

endmodule
