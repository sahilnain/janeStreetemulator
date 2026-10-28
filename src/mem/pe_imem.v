/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_imem: instruction memory, DEPTH x IW.
 *
 * One write port (host, via pe_regs) and one registered read port (core):
 * rdata is valid one cycle after raddr. Every implementation keeps this
 * timing, so an SRAM macro can replace it later without touching pe_core.
 *
 * IMPL:
 *   0 = flip-flops (default, safe for the flow)
 *   1 = latches (smaller; EXPERIMENTAL, not yet checked in the IHP flow).
 *       Write address/data are registered, and a row's latch is open
 *       while clk is low in the cycle after the write.
 *
 * The host may only write while all contexts are halted (enforced in
 * pe_regs), so no read/write collision logic is needed.
 */

`default_nettype none

module pe_imem #(
    parameter IW    = 24,
    parameter DEPTH = 64,
    parameter IMPL  = 0,
    // derived, do not override
    parameter AW    = (DEPTH > 1) ? $clog2(DEPTH) : 1
) (
    input  wire          clk,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [IW-1:0] wdata,
    input  wire [AW-1:0] raddr,
    output reg  [IW-1:0] rdata
);

  reg [IW-1:0] mem[0:DEPTH-1];

  always @(posedge clk) begin
    rdata <= mem[raddr];
  end

  generate
    if (IMPL == 0) begin : g_flop
      always @(posedge clk) begin
        if (we) mem[waddr] <= wdata;
      end
    end else begin : g_latch
      reg          we_q;
      reg [AW-1:0] waddr_q;
      reg [IW-1:0] wdata_q;

      always @(posedge clk) begin
        we_q    <= we;
        waddr_q <= waddr;
        wdata_q <= wdata;
      end

`ifdef FORMAL
      // (latch implementation, proved with sby `multiclock on`) clk is a free
      // signal that toggles on every solver step, so the latch's open/close
      // timing is modelled exactly. Same contract as the flop version: at
      // every rising edge, memory at f_addr holds the last value written, and
      // a read returns it one cycle later.
      (* gclk *) wire f_gclk;  // ticks once per solver step
      reg f_clk_q = 1'b0;
      always @(posedge f_gclk) f_clk_q <= clk;
      always @(*) assume (clk != f_clk_q);

      reg f_past_valid = 1'b0;
      always @(posedge clk) f_past_valid <= 1'b1;
      (* anyconst *) reg [AW-1:0] f_addr;
      reg [IW-1:0] f_val;
      always @(*) assume (f_addr < DEPTH);
      always @(*) if (!f_past_valid) assume (!we_q && mem[f_addr] == f_val);
      always @(posedge clk) if (we && waddr == f_addr) f_val <= wdata;
      always @(posedge clk) if (f_past_valid) assert (mem[f_addr] == f_val);
      always @(posedge clk)
        if (f_past_valid && $past(raddr) == f_addr) assert (rdata == $past(f_val));
`endif

      genvar i;
      for (i = 0; i < DEPTH; i = i + 1) begin : g_row
        wire open_row = ~clk & we_q & (waddr_q == i[AW-1:0]);
        /* verilator lint_off LATCH */
        always @(*) begin
          if (open_row) mem[i] = wdata_q;
        end
        /* verilator lint_on LATCH */
      end
    end
  endgenerate

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // (flop implementation; the latch proof is inside g_latch above) Track one
  // solver-chosen address f_addr: memory there always equals the last value
  // written, and a read of it returns that value one cycle later
  // (read-before-write on the same edge).
  generate
    if (IMPL == 0) begin : g_formal
      reg f_past_valid = 1'b0;
      always @(posedge clk) f_past_valid <= 1'b1;

      (* anyconst *) reg [AW-1:0] f_addr;
      reg [IW-1:0] f_val;

      always @(*) assume (f_addr < DEPTH);
      always @(*) if (!f_past_valid) assume (mem[f_addr] == f_val);
      always @(posedge clk) if (we && waddr == f_addr) f_val <= wdata;

      always @(*) assert (mem[f_addr] == f_val);
      always @(posedge clk)
        if (f_past_valid && $past(raddr) == f_addr) assert (rdata == $past(f_val));
    end
  endgenerate
`endif

endmodule
