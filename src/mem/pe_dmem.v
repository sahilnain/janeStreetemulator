/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_dmem: data memory, DEPTH x 8 flip-flops (docs/spec/blocks.md Section 8).
 *
 * One address, combinational read, one write per cycle. Two users:
 *   host (pe_regs DMEM_DATA access, h_acc = read or write strobe) has
 *   priority; the controller (ld / st) gets c_gnt = !h_acc and blocks
 *   otherwise. Address bits above log2(DEPTH) are ignored.
 */

`default_nettype none

module pe_dmem #(
    parameter DEPTH = 256,
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

  reg [7:0] mem[0:DEPTH-1];

  wire [7:0] a = h_acc ? h_addr : c_addr;
  wire [MW-1:0] ai = a[MW-1:0];
  wire we = h_acc ? h_we : c_we;
  wire [7:0] wd = h_acc ? h_wdata : c_wdata;

  assign c_gnt = ~h_acc;
  assign rdata = mem[ai];

  always @(posedge clk) begin
    if (we) mem[ai] <= wd;
  end

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
