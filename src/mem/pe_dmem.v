/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_dmem: data memory, DEPTH x 8, one port (docs/spec/blocks.md Section 8).
 * v3: it holds payload bytes only; pe_dmov arbitrates its users.
 *
 * One access per cycle, REGISTERED read: a read (re) in cycle N puts the
 * byte on rdata from cycle N+1 until the next read. A write (we) leaves rdata
 * alone. re and we are never both 1 (pe_dmov). This is how an SRAM macro
 * reads, and both implementations behave the same:
 *   IMPL = 0: flip-flops. CG = 1 clocks each byte by its own clock gate
 *             (pe_icg), enabled only by a write to that byte, so it holds
 *             without a 2:1 multiplexer per bit.
 *   IMPL = 1: the IHP SRAM macro RM_IHPSG13_1P_256x8_c3_bm_bist (DEPTH must
 *             be 256). Its BIST port is tied off and A_DLY is tied to 1, as
 *             its datasheet requires. Its contents are random at power-up.
 * Formal (sby) always uses IMPL = 0, CG = 0. Address bits above log2(DEPTH)
 * are ignored.
 */

`default_nettype none

module pe_dmem #(
    parameter DEPTH = 256,
    parameter CG    = 0,
    parameter IMPL  = 0,
    parameter MW    = (DEPTH > 1) ? $clog2(DEPTH) : 1
) (
    input  wire       clk,
    input  wire       re,
    input  wire       we,
    input  wire [7:0] addr,
    input  wire [7:0] wdata,
    output wire [7:0] rdata
);

  wire [MW-1:0] ai = addr[MW-1:0];

`ifdef FORMAL
  localparam USE_SRAM = 0;
  localparam USE_CG = 0;
`else
  localparam USE_SRAM = (IMPL != 0);
  localparam USE_CG = (CG != 0);
`endif

  generate
    if (USE_SRAM) begin : g_sram
      // The macro only exists in one size; refuse any other DEPTH at elaboration.
      if (DEPTH != 256) begin : g_bad_depth
        DMEM_IMPL_1_needs_DEPTH_256 u_err ();
      end
      RM_IHPSG13_1P_256x8_c3_bm_bist u_sram (
          .A_CLK      (clk),
          .A_MEN      (re | we),
          .A_WEN      (we),
          .A_REN      (re),
          .A_ADDR     (addr),
          .A_DIN      (wdata),
          .A_DLY      (1'b1),
          .A_DOUT     (rdata),
          .A_BM       (8'hFF),
          .A_BIST_CLK (1'b0),
          .A_BIST_EN  (1'b0),
          .A_BIST_MEN (1'b0),
          .A_BIST_WEN (1'b0),
          .A_BIST_REN (1'b0),
          .A_BIST_ADDR(8'd0),
          .A_BIST_DIN (8'd0),
          .A_BIST_BM  (8'd0)
      );
      wire _unused_s = &{1'b0, ai};
    end else begin : g_ff
      /* verilator lint_off UNDRIVEN */
      /* verilator lint_off UNUSEDSIGNAL */
      reg [7:0] mem[0:DEPTH-1];
      /* verilator lint_on UNUSEDSIGNAL */
      /* verilator lint_on UNDRIVEN */
      wire [7:0] rbyte[0:DEPTH-1];  // the byte at each address, whichever way it is stored
      reg [7:0] dout;

      always @(posedge clk) if (re) dout <= rbyte[ai];
      assign rdata = dout;

      genvar i;
      if (USE_CG) begin : g_cg
        for (i = 0; i < DEPTH; i = i + 1) begin : g_row
          wire gclk;
          pe_icg u_icg (
              .clk (clk),
              .en  (we & (ai == i[MW-1:0])),
              .gclk(gclk)
          );
          reg [7:0] q;
          always @(posedge gclk) q <= wdata;
          assign rbyte[i] = q;
        end
      end else begin : g_plain
        always @(posedge clk) begin
          if (we) mem[ai] <= wdata;
        end
        for (i = 0; i < DEPTH; i = i + 1) begin : g_row
          assign rbyte[i] = mem[i];
        end
      end

`ifdef FORMAL
      // One solver-chosen byte is tracked; reads return it as it was before
      // the edge, from the next cycle until the next read.
      (* anyconst *) reg [MW-1:0] f_addr;
      reg [7:0] f_val;
      reg f_pv = 1'b0;
      always @(posedge clk) f_pv <= 1'b1;
      always @(*) if (!f_pv) assume (mem[f_addr] == f_val);
      always @(*) assume (!(re && we));
      always @(posedge clk) if (we && ai == f_addr) f_val <= wdata;
      always @(*) assert (mem[f_addr] == f_val);
      always @(posedge clk) begin
        if (f_pv && $past(re) && $past(ai) == f_addr) assert (rdata == $past(f_val));
        if (f_pv && !$past(re)) assert (rdata == $past(rdata));
      end
      always @(*) cover (we && ai == f_addr && f_val != wdata);
`endif
    end
  endgenerate

  wire _unused = &{1'b0, addr};

endmodule
