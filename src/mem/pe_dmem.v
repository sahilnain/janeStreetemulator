/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_dmem: data memory, DEPTH x 8 flip-flops (docs/spec/blocks.md Section 8).
 *
 * CG = 1: each byte's flip-flops are clocked by their own clock gate (pe_icg),
 * enabled only by a write to that byte, so they hold without a 2:1
 * multiplexer per bit. Same behaviour as CG = 0. Formal (sby) uses CG = 0.
 *
 * One address, combinational read, one write per cycle. Two users:
 *   host (pe_regs DMEM_DATA access, h_acc = read or write strobe) has
 *   priority; the controller (ld / st) gets c_gnt = !h_acc and blocks
 *   otherwise. Address bits above log2(DEPTH) are ignored.
 */

`default_nettype none

module pe_dmem #(
    parameter DEPTH = 256,
    parameter CG    = 0,
    // derived, do not override
    parameter MW    = (DEPTH > 1) ? $clog2(DEPTH) : 1
) (
    input wire clk,

    input  wire       h_acc,
    input  wire       h_we,
    input  wire [7:0] h_addr,
    input  wire [7:0] h_wdata,
    input  wire       c_we,
    input  wire [7:0] c_addr,
    input  wire [7:0] c_wdata,
    output wire       c_gnt,
    output wire [7:0] rdata
);

  // Storage. With CG the bytes are separate registers (g_cg) and `mem` is unused.
  /* verilator lint_off UNDRIVEN */
  /* verilator lint_off UNUSEDSIGNAL */
  reg [7:0] mem[0:DEPTH-1];
  /* verilator lint_on UNUSEDSIGNAL */
  /* verilator lint_on UNDRIVEN */
  wire [7:0] rbyte[0:DEPTH-1];  // the byte at each address, whichever way it is stored

  wire [7:0] a = h_acc ? h_addr : c_addr;
  wire [MW-1:0] ai = a[MW-1:0];
  wire we = h_acc ? h_we : c_we;
  wire [7:0] wd = h_acc ? h_wdata : c_wdata;

  assign c_gnt = ~h_acc;
  assign rdata = rbyte[ai];

`ifdef FORMAL
  localparam USE_CG = 0;
`else
  localparam USE_CG = (CG != 0);
`endif

  generate
    if (USE_CG) begin : g_cg
      genvar i;
      for (i = 0; i < DEPTH; i = i + 1) begin : g_row
        wire gclk;
        pe_icg u_icg (
            .clk (clk),
            .en  (we & (ai == i[MW-1:0])),
            .gclk(gclk)
        );
        reg [7:0] q;
        always @(posedge gclk) q <= wd;
        assign rbyte[i] = q;
      end
    end else begin : g_ff
      always @(posedge clk) begin
        if (we) mem[ai] <= wd;
      end
      genvar i;
      for (i = 0; i < DEPTH; i = i + 1) begin : g_row
        assign rbyte[i] = mem[i];
      end
    end
  endgenerate

  wire _unused = &{1'b0, a};

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // Host priority (the controller is only granted when the host is idle; a
  // controller write in a host cycle has no effect), and data integrity:
  // the memory at a solver-chosen address always holds the last value
  // written there, and a read returns it.
  (* anyconst *) reg [MW-1:0] f_addr;
  reg [7:0] f_val;
  reg f_pv = 1'b0;
  always @(posedge clk) f_pv <= 1'b1;
  always @(*) if (!f_pv) assume (mem[f_addr] == f_val);
  always @(posedge clk) begin
    if (h_acc) begin
      if (h_we && h_addr[MW-1:0] == f_addr) f_val <= h_wdata;
    end else if (c_we && c_addr[MW-1:0] == f_addr) f_val <= c_wdata;
  end
  always @(*) begin
    assert (c_gnt == !h_acc);
    assert (mem[f_addr] == f_val);
    if (ai == f_addr) assert (rdata == f_val);
    assert (ai == (h_acc ? h_addr[MW-1:0] : c_addr[MW-1:0]));
    cover (h_acc && h_we && ai == f_addr && f_val != h_wdata);
  end
`endif

endmodule
