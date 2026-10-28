/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * tt_um_sahil_protoemu: Protocol Emulator top level (Tiny Tapeout wrapper).
 *
 * Pin map
 *   ui[3:0]  host data in        uo[3:0]  host data out
 *   ui[5:4]  host op             uo[4]    host ack (toggle)
 *   ui[6]    host strobe (toggle) uo[5]   IRQ
 *   ui[7]    reserved            uo[7:6]  reserved (0)
 *   uio[7:0] protocol pins P0-P7 (direction set per pin by firmware / open-drain)
 *
 * Architecture (Claude.md section 6):
 *   host bus -> pe_host_if -> pe_regs -> {pe_imem, config, FIFOs, status}
 *   pe_imem -> pe_core (contexts, decode, exec, vote) -> pe_pins -> uio
 *   pe_core <-> TX/RX pe_fifo <-> pe_regs
 */

`default_nettype none

module tt_um_sahil_protoemu (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // ------------------------------------------------- configuration
  // TODO(M0): generate these from config/*.yaml.
  localparam NUM_CTX = 2;
  localparam IMEM_DEPTH = 64;
  localparam IMEM_IMPL = 0;  // 0 = flops, 1 = latches
  localparam IW = 24;
  localparam NUM_PINS = 8;  // = uio width
  localparam DATA_W = 8;
  localparam FIFO_DEPTH = 4;
  localparam SR_W = 16;
  localparam CTR_W = 16;
  localparam VOTE_CTX_MASK = 2'b11;
  localparam VOTE_N_W = 4;
  localparam VOTE_S_W = 12;

  localparam AW = (IMEM_DEPTH > 1) ? $clog2(IMEM_DEPTH) : 1;
  localparam CTX_W = (NUM_CTX > 1) ? $clog2(NUM_CTX) : 1;
  localparam FIFO_CW = $clog2(FIFO_DEPTH + 1);

  // ---------------------------------------------------------- reset
  reg [1:0] rst_q;
  always @(posedge clk) rst_q <= {rst_q[0], ~rst_n};
  wire rst = rst_q[1];

  // --------------------------------------------------- host interface
  wire [7:0] r_addr, r_wdata, r_rdata;
  wire r_we, r_re, irq;

  pe_host_if u_host_if (
      .clk    (clk),
      .rst    (rst),
      .h_din  (ui_in[3:0]),
      .h_op   (ui_in[5:4]),
      .h_stb  (ui_in[6]),
      .h_dout (uo_out[3:0]),
      .h_ack  (uo_out[4]),
      .h_irq  (uo_out[5]),
      .r_addr (r_addr),
      .r_wdata(r_wdata),
      .r_we   (r_we),
      .r_re   (r_re),
      .r_rdata(r_rdata),
      .irq    (irq)
  );

  assign uo_out[7:6] = 2'b00;

  // ------------------------------------------------------ registers
  wire im_we;
  wire [AW-1:0] im_waddr, im_raddr;
  wire [IW-1:0] im_wdata, im_rdata;

  wire txf_wr, txf_rd, txf_full, txf_empty, txf_overrun, txf_clr;
  wire rxf_wr, rxf_rd, rxf_full, rxf_empty, rxf_overrun, rxf_clr;
  wire [DATA_W-1:0] txf_wdata, txf_rdata, rxf_wdata, rxf_rdata;
  wire [FIFO_CW-1:0] txf_count, rxf_count;

  wire [NUM_CTX-1:0] ctx_run, ctx_restart, ctx_shift_msb;
  wire [NUM_CTX*AW-1:0] ctx_start_pc, ctx_pc;
  wire [NUM_CTX*VOTE_N_W-1:0] vote_n, vote_k;
  wire [NUM_CTX*VOTE_S_W-1:0] vote_s;
  wire [NUM_CTX-1:0] ctx_ready, ctx_vote_busy, ctx_c, ctx_to;

  wire [NUM_PINS*CTX_W-1:0] pin_owner;
  wire [NUM_PINS-1:0] pin_od, pin_in;
  wire owner_err, owner_clr;

  pe_regs #(
      .NUM_CTX   (NUM_CTX),
      .IMEM_DEPTH(IMEM_DEPTH),
      .IW        (IW),
      .NUM_PINS  (NUM_PINS),
      .DATA_W    (DATA_W),
      .FIFO_CW   (FIFO_CW),
      .VOTE_N_W  (VOTE_N_W),
      .VOTE_S_W  (VOTE_S_W)
  ) u_regs (
      .clk            (clk),
      .rst            (rst),
      .r_addr         (r_addr),
      .r_wdata        (r_wdata),
      .r_we           (r_we),
      .r_re           (r_re),
      .r_rdata        (r_rdata),
      .irq            (irq),
      .im_we          (im_we),
      .im_waddr       (im_waddr),
      .im_wdata       (im_wdata),
      .txf_wr         (txf_wr),
      .txf_wdata      (txf_wdata),
      .txf_full       (txf_full),
      .txf_empty      (txf_empty),
      .txf_count      (txf_count),
      .txf_overrun    (txf_overrun),
      .txf_clr_overrun(txf_clr),
      .rxf_rd         (rxf_rd),
      .rxf_rdata      (rxf_rdata),
      .rxf_full       (rxf_full),
      .rxf_empty      (rxf_empty),
      .rxf_count      (rxf_count),
      .rxf_overrun    (rxf_overrun),
      .rxf_clr_overrun(rxf_clr),
      .ctx_run        (ctx_run),
      .ctx_restart    (ctx_restart),
      .ctx_start_pc   (ctx_start_pc),
      .ctx_shift_msb  (ctx_shift_msb),
      .vote_n         (vote_n),
      .vote_k         (vote_k),
      .vote_s         (vote_s),
      .ctx_pc         (ctx_pc),
      .ctx_ready      (ctx_ready),
      .ctx_vote_busy  (ctx_vote_busy),
      .ctx_c          (ctx_c),
      .ctx_to         (ctx_to),
      .pin_owner      (pin_owner),
      .pin_od         (pin_od),
      .pin_in         (pin_in),
      .owner_err      (owner_err),
      .owner_clr_err  (owner_clr)
  );

  // ------------------------------------------------- instruction memory
  pe_imem #(
      .IW   (IW),
      .DEPTH(IMEM_DEPTH),
      .IMPL (IMEM_IMPL)
  ) u_imem (
      .clk  (clk),
      .we   (im_we),
      .waddr(im_waddr),
      .wdata(im_wdata),
      .raddr(im_raddr),
      .rdata(im_rdata)
  );

  // ------------------------------------------------------------ FIFOs
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
      .clr_overrun(txf_clr)
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

  // ------------------------------------------------------------- core
  wire pin_wr_en, pin_oe_en;
  wire [CTX_W-1:0] pin_wr_ctx;
  wire [NUM_PINS-1:0] pin_wr_mask, pin_wr_val, pin_oe_mask, pin_oe_val;

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

  // ------------------------------------------------------- pin fabric
`ifdef FORMAL
  wire [NUM_PINS-1:0] f_pins_out, f_pins_oe;
`endif
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
      .pad_out  (uio_out),
      .pad_oe   (uio_oe),
      .pad_in   (uio_in)
`ifdef FORMAL
      ,
      .f_out_r  (f_pins_out),
      .f_oe_r   (f_pins_oe)
`endif
  );

  wire _unused = &{ena, ui_in[7], 1'b0};

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // Whole-chip proof. Every sub-module's properties are included; their
  // input contracts are ASSERTED here (only the host-bus protocol in
  // pe_host_if stays an assumption, because those inputs are chip pins).
  // Chip-level properties, unbounded (k-induction):
  // * uo_out[7:6] (reserved) are always 0.
  // * Pins are only written by a context that is running.
  // * Instruction memory is never written while any context runs.
  // * Until a context has been started (RUN != 0) after reset, no protocol
  //   pin drives: uio_oe == 0 and the pin registers hold their reset value.
  // Covers (cover task): from reset, driving ONLY the host-bus pins, a
  // context can be started and a protocol pin can be driven.
  always @(*) assume (ena);
  reg [1:0] f_cyc = 2'd0;
  always @(posedge clk) if (f_cyc != 2'd3) f_cyc <= f_cyc + 1'b1;
  always @(*) if (f_cyc < 2'd3) assume (!rst_n);

  reg f_rst_seen = 1'b0;
  always @(posedge clk) if (rst) f_rst_seen <= 1'b1;
  wire f_ok = f_rst_seen & ~rst;

  reg f_ever_run;
  always @(posedge clk) begin
    if (rst) f_ever_run <= 1'b0;
    else if (ctx_run != {NUM_CTX{1'b0}}) f_ever_run <= 1'b1;
  end

  always @(*) begin
    assert (uo_out[7:6] == 2'b00);
    if (f_ok) begin
      if (pin_wr_en || pin_oe_en) assert (ctx_run[pin_wr_ctx]);
      if (im_we) assert (ctx_run == {NUM_CTX{1'b0}});
      if (!f_ever_run) begin
        assert (f_pins_out == {NUM_PINS{1'b1}} && f_pins_oe == {NUM_PINS{1'b0}});
        assert (uio_oe == 8'h00);
      end
      cover (ctx_run != {NUM_CTX{1'b0}});
      cover (uio_oe != 8'h00);
    end
  end
`endif

endmodule
