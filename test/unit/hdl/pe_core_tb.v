/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_core_tb: test harness for pe_core (unit test only, not part of the chip).
 *
 * What this file does
 * -------------------
 * pe_core cannot run alone: it needs instruction memory, FIFOs and the pin
 * fabric. This harness wires the REAL pe_imem, two pe_fifo and pe_pins
 * around pe_core, exactly as the top level does, but without the host bus
 * and register file. The cocotb test drives the imem write port, the host
 * sides of the FIFOs, and the context/pin configuration directly.
 */

`default_nettype none
`timescale 1ns / 1ps

module pe_core_tb #(
    parameter NUM_CTX       = 2,
    parameter IMEM_DEPTH    = 64,
    parameter IW            = 24,
    parameter NUM_PINS      = 8,
    parameter SR_W          = 16,
    parameter DATA_W        = 8,
    parameter CTR_W         = 16,
    parameter FIFO_DEPTH    = 4,
    parameter VOTE_CTX_MASK = 3,
    parameter VOTE_N_W      = 4,
    parameter VOTE_S_W      = 12,
    parameter AW            = (IMEM_DEPTH > 1) ? $clog2(IMEM_DEPTH) : 1,
    parameter CTX_W         = (NUM_CTX > 1) ? $clog2(NUM_CTX) : 1,
    parameter FIFO_CW       = $clog2(FIFO_DEPTH + 1)
) (
    input wire clk,
    input wire rst,

    // imem write port
    input wire          im_we,
    input wire [AW-1:0] im_waddr,
    input wire [IW-1:0] im_wdata,

    // host side of FIFOs
    input  wire               txf_wr,
    input  wire [ DATA_W-1:0] txf_wdata,
    output wire               txf_full,
    output wire               txf_empty,
    input  wire               rxf_rd,
    output wire [ DATA_W-1:0] rxf_rdata,
    output wire               rxf_empty,
    output wire               rxf_full,
    output wire [FIFO_CW-1:0] rxf_count,
    output wire               rxf_overrun,
    input  wire               rxf_clr,

    // context config
    input wire [        NUM_CTX-1:0] ctx_run,
    input wire [        NUM_CTX-1:0] ctx_restart,
    input wire [     NUM_CTX*AW-1:0] ctx_start_pc,
    input wire [        NUM_CTX-1:0] ctx_shift_msb,
    input wire [NUM_CTX*VOTE_N_W-1:0] vote_n,
    input wire [NUM_CTX*VOTE_N_W-1:0] vote_k,
    input wire [NUM_CTX*VOTE_S_W-1:0] vote_s,

    // status
    output wire [NUM_CTX*AW-1:0] ctx_pc,
    output wire [   NUM_CTX-1:0] ctx_ready,
    output wire [   NUM_CTX-1:0] ctx_vote_busy,
    output wire [   NUM_CTX-1:0] ctx_c,
    output wire [   NUM_CTX-1:0] ctx_to,

    // pins
    input  wire [NUM_PINS*CTX_W-1:0] pin_owner,
    input  wire [      NUM_PINS-1:0] pin_od,
    output wire                      owner_err,
    input  wire                      owner_clr,
    output wire [      NUM_PINS-1:0] pad_out,
    output wire [      NUM_PINS-1:0] pad_oe,
    input  wire [      NUM_PINS-1:0] pad_in
);

  wire [AW-1:0] im_raddr;
  wire [IW-1:0] im_rdata;

  wire txf_rd, txf_overrun, rxf_wr;
  wire [DATA_W-1:0] txf_rdata, rxf_wdata;
  wire [FIFO_CW-1:0] txf_count;

  wire pin_wr_en, pin_oe_en;
  wire [CTX_W-1:0] pin_wr_ctx;
  wire [NUM_PINS-1:0] pin_wr_mask, pin_wr_val, pin_oe_mask, pin_oe_val, pin_in;

  pe_imem #(
      .IW   (IW),
      .DEPTH(IMEM_DEPTH)
  ) u_imem (
      .clk  (clk),
      .we   (im_we),
      .waddr(im_waddr),
      .wdata(im_wdata),
      .raddr(im_raddr),
      .rdata(im_rdata)
  );

  pe_fifo #(
      .W    (DATA_W),
      .DEPTH(FIFO_DEPTH)
  ) u_txf (
      .clk        (clk),
      .rst        (rst),
      .wr         (txf_wr),
      .wdata      (txf_wdata),
      .full       (txf_full),
      .rd         (txf_rd),
      .rdata      (txf_rdata),
      .empty      (txf_empty),
      .count      (txf_count),
      .overrun    (txf_overrun),
      .clr_overrun(1'b0)
  );

  pe_fifo #(
      .W    (DATA_W),
      .DEPTH(FIFO_DEPTH)
  ) u_rxf (
      .clk        (clk),
      .rst        (rst),
      .wr         (rxf_wr),
      .wdata      (rxf_wdata),
      .full       (rxf_full),
      .rd         (rxf_rd),
      .rdata      (rxf_rdata),
      .empty      (rxf_empty),
      .count      (rxf_count),
      .overrun    (rxf_overrun),
      .clr_overrun(rxf_clr)
  );

  pe_core #(
      .NUM_CTX      (NUM_CTX),
      .IMEM_DEPTH   (IMEM_DEPTH),
      .IW           (IW),
      .NUM_PINS     (NUM_PINS),
      .SR_W         (SR_W),
      .DATA_W       (DATA_W),
      .CTR_W        (CTR_W),
      .VOTE_CTX_MASK(VOTE_CTX_MASK),
      .VOTE_N_W     (VOTE_N_W),
      .VOTE_S_W     (VOTE_S_W)
  ) u_core (
      .clk          (clk),
      .rst          (rst),
      .im_raddr     (im_raddr),
      .im_rdata     (im_rdata),
      .pin_wr_en    (pin_wr_en),
      .pin_wr_ctx   (pin_wr_ctx),
      .pin_wr_mask  (pin_wr_mask),
      .pin_wr_val   (pin_wr_val),
      .pin_oe_en    (pin_oe_en),
      .pin_oe_mask  (pin_oe_mask),
      .pin_oe_val   (pin_oe_val),
      .pin_in       (pin_in),
      .txf_rd       (txf_rd),
      .txf_rdata    (txf_rdata),
      .txf_empty    (txf_empty),
      .rxf_wr       (rxf_wr),
      .rxf_wdata    (rxf_wdata),
      .rxf_full     (rxf_full),
      .ctx_run      (ctx_run),
      .ctx_restart  (ctx_restart),
      .ctx_start_pc (ctx_start_pc),
      .ctx_shift_msb(ctx_shift_msb),
      .vote_n       (vote_n),
      .vote_k       (vote_k),
      .vote_s       (vote_s),
      .ctx_pc       (ctx_pc),
      .ctx_ready    (ctx_ready),
      .ctx_vote_busy(ctx_vote_busy),
      .ctx_c        (ctx_c),
      .ctx_to       (ctx_to)
  );

  pe_pins #(
      .NUM_PINS(NUM_PINS),
      .NUM_CTX (NUM_CTX)
  ) u_pins (
      .clk      (clk),
      .rst      (rst),
      .wr_en    (pin_wr_en),
      .wr_ctx   (pin_wr_ctx),
      .wr_mask  (pin_wr_mask),
      .wr_val   (pin_wr_val),
      .oe_en    (pin_oe_en),
      .oe_mask  (pin_oe_mask),
      .oe_val   (pin_oe_val),
      .pin_in   (pin_in),
      .pin_owner(pin_owner),
      .pin_od   (pin_od),
      .owner_err(owner_err),
      .clr_err  (owner_clr),
      .pad_out  (pad_out),
      .pad_oe   (pad_oe),
      .pad_in   (pad_in)
  );

  wire _unused = &{1'b0, txf_overrun, txf_count};

endmodule
