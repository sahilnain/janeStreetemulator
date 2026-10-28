/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_regs: register file and address decode (8-bit address, 8-bit data).
 *
 * Global registers
 *   0x00 ID        R   constant ID
 *   0x01 VERSION   R   constant version
 *   0x02 RUN       RW  bit c = context c runs
 *   0x03 RESTART   W   write 1 to bit c: context c PC <- START_PC, state cleared
 *   0x04 STATUS    R   [0] txf_empty [1] txf_full [2] rxf_empty [3] rxf_full
 *                      [4] txf_overrun [5] rxf_overrun [6] owner_err [7] imem_err
 *   0x05 ERR_CLR   W   write 1 to clear: [4] txf_overrun [5] rxf_overrun
 *                      [6] owner_err [7] imem_err
 *   0x06 IRQ_EN    RW  [0] RX FIFO not empty [1] TX FIFO not full [2] any error
 *   0x07 FIFO_CNT  R   [3:0] TX FIFO count, [7:4] RX FIFO count
 *   0x08 TXF_DATA  W   push a byte into the TX FIFO (firmware `pull`s it)
 *   0x09 RXF_DATA  R   pop a byte from the RX FIFO (firmware `push`ed it)
 *   0x0A IMEM_ADDR RW  word address for the next IMEM commit (auto-increments)
 *   0x0B PIN_IN    R   synchronised pin inputs [7:0]
 *   0x0C PIN_IN_H  R   synchronised pin inputs [15:8]
 *   0x10-0x13 IMEM_DATA W  instruction staging, byte 0 = bits [7:0]. Writing
 *                      byte NB-1 (NB = ceil(IW/8), 0x12 for IW=24) commits the
 *                      word to IMEM_ADDR. A commit while any context runs is
 *                      dropped and sets imem_err.
 *   0x14-0x15 PIN_OD   RW  open-drain enable, bit i = pin i
 *   0x18-0x1F PIN_OWNER RW packed owner map, CTX_W bits per pin, pin 0 at bit 0
 *
 * Per-context registers, base = 0x20 + 8*c
 *   +0 START_PC   RW
 *   +1 CFG        RW  [0] shift_msb (0 = LSB-first)
 *   +2 VOTE_N     RW  samples per vote
 *   +3 VOTE_K     RW  samples ignored at each end
 *   +4 VOTE_S_LO  RW  cycles per sample [7:0]
 *   +5 VOTE_S_HI  RW  cycles per sample [15:8]
 *   +6 PC         R
 *   +7 CSTAT      R   [0] ready [1] vote_busy [2] C [3] TO [4] run
 *
 * Unmapped addresses read 0. Limits: NUM_CTX <= 8, NUM_PINS <= 16,
 * IW <= 32, NUM_PINS*CTX_W <= 64, VOTE_N_W <= 8, VOTE_S_W <= 16, FIFO count <= 4 bits.
 */

`default_nettype none

module pe_regs #(
    parameter NUM_CTX    = 2,
    parameter IMEM_DEPTH = 64,
    parameter IW         = 24,
    parameter NUM_PINS   = 8,
    parameter DATA_W     = 8,
    parameter FIFO_CW    = 3,
    parameter VOTE_N_W   = 4,
    parameter VOTE_S_W   = 12,
    parameter VOTE_N_RST = 9,
    parameter VOTE_K_RST = 1,
    parameter VOTE_S_RST = 48,   // 9 x 48 = 432 cycles, about 115200 baud at 50 MHz
    parameter ID         = 8'h50,
    parameter VERSION    = 8'h01,
    // derived, do not override
    parameter AW         = (IMEM_DEPTH > 1) ? $clog2(IMEM_DEPTH) : 1,
    parameter CTX_W      = (NUM_CTX > 1) ? $clog2(NUM_CTX) : 1
) (
    input wire clk,
    input wire rst,

    // register bus from pe_host_if
    input  wire [7:0] r_addr,
    input  wire [7:0] r_wdata,
    input  wire       r_we,
    input  wire       r_re,
    output reg  [7:0] r_rdata,
    output wire       irq,

    // instruction memory write port
    output wire          im_we,
    output wire [AW-1:0] im_waddr,
    output reg  [IW-1:0] im_wdata,

    // TX FIFO (host writes)
    output wire               txf_wr,
    output wire [ DATA_W-1:0] txf_wdata,
    input  wire               txf_full,
    input  wire               txf_empty,
    input  wire [FIFO_CW-1:0] txf_count,
    input  wire               txf_overrun,
    output wire               txf_clr_overrun,

    // RX FIFO (host reads)
    output wire               rxf_rd,
    input  wire [ DATA_W-1:0] rxf_rdata,
    input  wire               rxf_full,
    input  wire               rxf_empty,
    input  wire [FIFO_CW-1:0] rxf_count,
    input  wire               rxf_overrun,
    output wire               rxf_clr_overrun,

    // core configuration
    output reg  [        NUM_CTX-1:0] ctx_run,
    output reg  [        NUM_CTX-1:0] ctx_restart,
    output reg  [     NUM_CTX*AW-1:0] ctx_start_pc,
    output reg  [        NUM_CTX-1:0] ctx_shift_msb,
    output reg  [NUM_CTX*VOTE_N_W-1:0] vote_n,
    output reg  [NUM_CTX*VOTE_N_W-1:0] vote_k,
    output reg  [NUM_CTX*VOTE_S_W-1:0] vote_s,

    // core status
    input wire [NUM_CTX*AW-1:0] ctx_pc,
    input wire [   NUM_CTX-1:0] ctx_ready,
    input wire [   NUM_CTX-1:0] ctx_vote_busy,
    input wire [   NUM_CTX-1:0] ctx_c,
    input wire [   NUM_CTX-1:0] ctx_to,

    // pin fabric
    output reg  [NUM_PINS*CTX_W-1:0] pin_owner,
    output reg  [      NUM_PINS-1:0] pin_od,
    input  wire [      NUM_PINS-1:0] pin_in,
    input  wire                      owner_err,
    output wire                      owner_clr_err
);

  // Addresses. Localparams are 32-bit integers, so constant arithmetic such
  // as A_IMDATA + b/8 does not overflow.
  localparam A_ID = 'h00;
  localparam A_VERSION = 'h01;
  localparam A_RUN = 'h02;
  localparam A_RESTART = 'h03;
  localparam A_STATUS = 'h04;
  localparam A_ERR_CLR = 'h05;
  localparam A_IRQ_EN = 'h06;
  localparam A_FIFO_CNT = 'h07;
  localparam A_TXF = 'h08;
  localparam A_RXF = 'h09;
  localparam A_IMADDR = 'h0A;
  localparam A_PIN_IN = 'h0B;
  localparam A_IMDATA = 'h10;
  localparam A_PIN_OD = 'h14;
  localparam A_OWNER = 'h18;
  localparam A_CTX0 = 'h20;

  localparam NB = (IW + 7) / 8;
  localparam OWN_W = NUM_PINS * CTX_W;
  localparam CW4 = (FIFO_CW < 4) ? FIFO_CW : 4;

  wire [31:0] addr = {24'd0, r_addr};

  reg [AW-1:0] imaddr;
  reg [IW-1:0] stage;
  reg [   2:0] irq_en;
  reg          imem_err;

  wire wr = r_we;
  wire wr_errclr = wr & (addr == A_ERR_CLR);
  wire any_run = |ctx_run;
  wire im_commit = wr & (addr == A_IMDATA + NB - 1);

  assign im_we           = im_commit & ~any_run;
  assign im_waddr        = imaddr;
  assign txf_wr          = wr & (addr == A_TXF);
  assign txf_wdata       = r_wdata[DATA_W-1:0];
  assign rxf_rd          = r_re & (addr == A_RXF);
  assign txf_clr_overrun = wr_errclr & r_wdata[4];
  assign rxf_clr_overrun = wr_errclr & r_wdata[5];
  assign owner_clr_err   = wr_errclr & r_wdata[6];

  wire any_err = txf_overrun | rxf_overrun | owner_err | imem_err;
  assign irq = (irq_en[0] & ~rxf_empty) | (irq_en[1] & ~txf_full) | (irq_en[2] & any_err);

  integer b, c;

  // Committed IMEM word = staging bytes plus the byte being written now.
  always @(*) begin
    for (b = 0; b < IW; b = b + 1) begin
      if (b / 8 == NB - 1) im_wdata[b] = r_wdata[b%8];
      else im_wdata[b] = stage[b];
    end
  end

  // Restart is a one-cycle pulse.
  always @(*) begin
    for (c = 0; c < NUM_CTX; c = c + 1) ctx_restart[c] = wr & (addr == A_RESTART) & r_wdata[c];
  end

  // ------------------------------------------------------------- writes
  always @(posedge clk) begin
    if (rst) begin
      ctx_run       <= {NUM_CTX{1'b0}};
      ctx_shift_msb <= {NUM_CTX{1'b0}};
      ctx_start_pc  <= {(NUM_CTX * AW) {1'b0}};
      for (c = 0; c < NUM_CTX; c = c + 1) begin
        vote_n[c*VOTE_N_W+:VOTE_N_W] <= VOTE_N_RST[VOTE_N_W-1:0];
        vote_k[c*VOTE_N_W+:VOTE_N_W] <= VOTE_K_RST[VOTE_N_W-1:0];
        vote_s[c*VOTE_S_W+:VOTE_S_W] <= VOTE_S_RST[VOTE_S_W-1:0];
      end
      pin_owner <= {OWN_W{1'b0}};
      pin_od    <= {NUM_PINS{1'b0}};
      imaddr    <= {AW{1'b0}};
      stage     <= {IW{1'b0}};
      irq_en    <= 3'b000;
      imem_err  <= 1'b0;
    end else begin
      if (wr) begin
        for (b = 0; b < NUM_CTX; b = b + 1) if (addr == A_RUN) ctx_run[b] <= r_wdata[b];
        if (addr == A_IRQ_EN) irq_en <= r_wdata[2:0];
        for (b = 0; b < IW; b = b + 1) if (addr == A_IMDATA + b / 8) stage[b] <= r_wdata[b%8];
        for (b = 0; b < NUM_PINS; b = b + 1) if (addr == A_PIN_OD + b / 8) pin_od[b] <= r_wdata[b%8];
        for (b = 0; b < OWN_W; b = b + 1) if (addr == A_OWNER + b / 8) pin_owner[b] <= r_wdata[b%8];

        for (c = 0; c < NUM_CTX; c = c + 1) begin
          for (b = 0; b < AW; b = b + 1)
          if (addr == A_CTX0 + 8 * c + 0) ctx_start_pc[c*AW+b] <= r_wdata[b];
          if (addr == A_CTX0 + 8 * c + 1) ctx_shift_msb[c] <= r_wdata[0];
          for (b = 0; b < VOTE_N_W; b = b + 1) begin
            if (addr == A_CTX0 + 8 * c + 2) vote_n[c*VOTE_N_W+b] <= r_wdata[b];
            if (addr == A_CTX0 + 8 * c + 3) vote_k[c*VOTE_N_W+b] <= r_wdata[b];
          end
          for (b = 0; b < VOTE_S_W; b = b + 1)
          if (addr == A_CTX0 + 8 * c + 4 + b / 8) vote_s[c*VOTE_S_W+b] <= r_wdata[b%8];
        end
      end

      // IMEM address: written by the host, or auto-incremented on commit.
      if (wr && addr == A_IMADDR) imaddr <= r_wdata[AW-1:0];
      else if (im_we) imaddr <= imaddr + 1'b1;

      if (im_commit && any_run) imem_err <= 1'b1;
      else if (wr_errclr && r_wdata[7]) imem_err <= 1'b0;
    end
  end

  // ------------------------------------------------------------ readback
  always @(*) begin
    r_rdata = 8'd0;
    case (addr)
      A_ID: r_rdata = ID[7:0];
      A_VERSION: r_rdata = VERSION[7:0];
      A_STATUS:
      r_rdata = {
        imem_err, owner_err, rxf_overrun, txf_overrun, rxf_full, rxf_empty, txf_full, txf_empty
      };
      A_IRQ_EN: r_rdata = {5'd0, irq_en};
      A_RXF: r_rdata[DATA_W-1:0] = rxf_rdata;
      default: ;
    endcase

    for (b = 0; b < CW4; b = b + 1) begin
      if (addr == A_FIFO_CNT) r_rdata[b] = txf_count[b];
      if (addr == A_FIFO_CNT) r_rdata[4+b] = rxf_count[b];
    end
    for (b = 0; b < NUM_CTX; b = b + 1) if (addr == A_RUN) r_rdata[b] = ctx_run[b];
    for (b = 0; b < AW; b = b + 1) if (addr == A_IMADDR) r_rdata[b] = imaddr[b];
    for (b = 0; b < NUM_PINS; b = b + 1) begin
      if (addr == A_PIN_IN + b / 8) r_rdata[b%8] = pin_in[b];
      if (addr == A_PIN_OD + b / 8) r_rdata[b%8] = pin_od[b];
    end
    for (b = 0; b < OWN_W; b = b + 1) if (addr == A_OWNER + b / 8) r_rdata[b%8] = pin_owner[b];

    for (c = 0; c < NUM_CTX; c = c + 1) begin
      for (b = 0; b < AW; b = b + 1) begin
        if (addr == A_CTX0 + 8 * c + 0) r_rdata[b] = ctx_start_pc[c*AW+b];
        if (addr == A_CTX0 + 8 * c + 6) r_rdata[b] = ctx_pc[c*AW+b];
      end
      if (addr == A_CTX0 + 8 * c + 1) r_rdata[0] = ctx_shift_msb[c];
      for (b = 0; b < VOTE_N_W; b = b + 1) begin
        if (addr == A_CTX0 + 8 * c + 2) r_rdata[b] = vote_n[c*VOTE_N_W+b];
        if (addr == A_CTX0 + 8 * c + 3) r_rdata[b] = vote_k[c*VOTE_N_W+b];
      end
      for (b = 0; b < VOTE_S_W; b = b + 1)
      if (addr == A_CTX0 + 8 * c + 4 + b / 8) r_rdata[b%8] = vote_s[c*VOTE_S_W+b];
      if (addr == A_CTX0 + 8 * c + 7)
        r_rdata[4:0] = {ctx_run[c], ctx_to[c], ctx_c[c], ctx_vote_busy[c], ctx_ready[c]};
    end
  end

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // * IMEM is written only by a commit to the last IMEM_DATA byte while no
  //   context runs; a commit while running sets imem_err (sticky until
  //   ERR_CLR bit 7) and IMEM_ADDR auto-increments only on a real write.
  // * FIFO strobes, RESTART and ERR_CLR pulses happen only for their own
  //   address and access type.
  // * No write -> no configuration register changes (reads have no side
  //   effects except popping RXF); a write only changes the register at
  //   its address (checked for RUN, PIN_OWNER, PIN_OD, per-context block).
  // * irq follows IRQ_EN exactly.
`ifdef PE_REGS_TOP
  initial assume (rst);
`endif
  reg f_rst_seen = 1'b0;
  always @(posedge clk) if (rst) f_rst_seen <= 1'b1;
  wire f_ok = f_rst_seen & ~rst;
  reg  f_past_ok = 1'b0;
  always @(posedge clk) f_past_ok <= f_ok;

  always @(*) begin
    if (f_ok) begin
      if (im_we) assert (r_we && addr == A_IMDATA + NB - 1 && ctx_run == {NUM_CTX{1'b0}});
      assert (txf_wr == (r_we && addr == A_TXF));
      assert (rxf_rd == (r_re && addr == A_RXF));
      if (ctx_restart != {NUM_CTX{1'b0}}) assert (r_we && addr == A_RESTART);
      if (txf_clr_overrun || rxf_clr_overrun || owner_clr_err) assert (r_we && addr == A_ERR_CLR);
      assert (irq == ((irq_en[0] & ~rxf_empty) | (irq_en[1] & ~txf_full) | (irq_en[2] & any_err)));
    end
  end

  always @(posedge clk) begin
    if (f_past_ok && f_ok) begin
      if (!$past(r_we)) begin
        assert (ctx_run == $past(ctx_run) && ctx_shift_msb == $past(ctx_shift_msb));
        assert (ctx_start_pc == $past(ctx_start_pc) && vote_n == $past(vote_n));
        assert (vote_k == $past(vote_k) && vote_s == $past(vote_s));
        assert (pin_owner == $past(pin_owner) && pin_od == $past(pin_od));
        assert (irq_en == $past(irq_en) && stage == $past(stage) && imaddr == $past(imaddr));
      end else begin
        if ($past(addr) != A_RUN) assert (ctx_run == $past(ctx_run));
        else assert (ctx_run == $past(r_wdata[NUM_CTX-1:0]));
        if ($past(addr) < A_OWNER || $past(addr) >= A_OWNER + 8) assert (pin_owner == $past(pin_owner));
        if ($past(addr) != A_PIN_OD && $past(addr) != A_PIN_OD + 1) assert (pin_od == $past(pin_od));
        if ($past(addr) < A_CTX0)
          assert (ctx_start_pc == $past(ctx_start_pc) && vote_n == $past(vote_n) && vote_k == $past(vote_k)
                  && vote_s == $past(vote_s) && ctx_shift_msb == $past(ctx_shift_msb));
      end
      if ($past(r_we) && $past(addr) == A_IMADDR) assert (imaddr == $past(r_wdata[AW-1:0]));
      else if ($past(im_we)) assert (imaddr == $past(imaddr) + 1'b1);
      else assert (imaddr == $past(imaddr));
      if ($past(im_commit && any_run)) assert (imem_err);
      else if ($past(wr_errclr && r_wdata[7])) assert (!imem_err);
      else assert (imem_err == $past(imem_err));
    end
  end
`endif

endmodule
