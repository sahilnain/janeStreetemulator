/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_regs: host register map, v2 (docs/spec/blocks.md Section 9).
 *
 *   0x00 ID        R   constant ID
 *   0x01 VERSION   R   0x02
 *   0x02 RUN       RW  [0] controller runs
 *   0x03 RESTART   W   [0] = 1: restart the controller, units and detectors
 *   0x04 STATUS    R   [0] txf_empty [1] txf_full [2] rxf_empty [3] rxf_full
 *                      [4] txf_overrun [5] rxf_overrun [6] owner_err [7] imem_err
 *   0x05 ERR_CLR   W   write 1 to clear STATUS bits 4-7
 *   0x06 IRQ_EN    RW  [0] RXF not empty [1] TXF not full [2] any error
 *                      [3] controller IRQ (`host irq`)
 *   0x07 FIFO_CNT  R   [3:0] TX count, [7:4] RX count
 *   0x08 TXF_DATA  W   push to the TX FIFO
 *   0x09 RXF_DATA  R   pop from the RX FIFO
 *   0x0A IMEM_ADDR RW  word address [7:0] (auto-increments on commit)
 *   0x0B PIN_IN    R   pin_in[7:0]     0x0C PIN_IN_H R pin_in[15:8]
 *   0x0D IMEM_ADDR_H RW word address [9:8]
 *   0x0E STATUS2   R/W1C [0] ill_err [1] stk_err [2] rx_overrun
 *                      [3] tx_underrun [4] ctrl_irq
 *   0x0F DOORBELL  W   any write sets the controller's DB flag
 *   0x10-0x13 IMEM_DATA W instruction staging, byte 0 = bits [7:0]; writing
 *                      0x13 commits to IMEM_ADDR. A commit while RUN = 1 is
 *                      dropped and sets imem_err.
 *   0x14-0x15 PIN_OD   RW  open-drain enable, bit i = pin i
 *   0x18-0x1B PIN_OWNER RW 2 bits per pin, pin 0 at bit 0
 *   0x20-0x21 START_PC RW  0x22-0x23 PC R
 *   0x24 CSTAT     R   [0] run [1] blocked [2] Z [3] C [4] TO [5] ETX [6] ERX [7] DB
 *   0x28 DMEM_ADDR RW  0x29 DMEM_DATA RW (each access: DMEM_ADDR + 1)
 *
 * Unmapped addresses read 0. Limits: AW <= 10, NUM_PINS <= 16, FIFO count
 * <= 4 bits shown.
 */

`default_nettype none

module pe_regs #(
    parameter IMEM_DEPTH = 64,
    parameter NUM_PINS   = 8,
    parameter FIFO_CW    = 4,
    parameter ID         = 8'h50,
    parameter VERSION    = 8'h02,
    // derived, do not override
    parameter AW         = (IMEM_DEPTH > 1) ? $clog2(IMEM_DEPTH) : 1
) (
    input wire clk,
    input wire rst,
`ifdef PE_CHIP_FORMAL
    input wire f_seen_i,
`endif

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
    output reg  [  31:0] im_wdata,

    // TX FIFO (host writes)
    output wire               txf_wr,
    output wire [        7:0] txf_wdata,
    input  wire               txf_full,
    input  wire               txf_empty,
    input  wire [FIFO_CW-1:0] txf_count,
    input  wire               txf_overrun,
    output wire               txf_clr_overrun,

    // RX FIFO (host reads)
    output wire               rxf_rd,
    input  wire [        7:0] rxf_rdata,
    input  wire               rxf_full,
    input  wire               rxf_empty,
    input  wire [FIFO_CW-1:0] rxf_count,
    input  wire               rxf_overrun,
    output wire               rxf_clr_overrun,

    // controller
    output reg           run,
    output wire          restart,
    output reg  [AW-1:0] start_pc,
    output wire          db_set,
    input  wire [AW-1:0] pc,
    input  wire [   7:0] cstat_in,     // [1] blocked [2] Z [3] C [4] TO [5] ETX [6] ERX [7] DB
    input  wire          ill_err,
    input  wire          stk_err,
    input  wire          tx_underrun,
    input  wire          ctrl_irq,
    input  wire          rx_overrun,
    output wire          rx_ovr_clr,

    // pin fabric
    output reg  [2*NUM_PINS-1:0] pin_owner,
    output reg  [  NUM_PINS-1:0] pin_od,
    input  wire [  NUM_PINS-1:0] pin_in,
    input  wire                  owner_err,
    output wire                  owner_clr_err,

    // data memory (host side)
    output wire       dm_acc,
    output wire       dm_we,
    output reg  [7:0] dm_addr,
    output wire [7:0] dm_wdata,
    input  wire [7:0] dm_rdata
);

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
  localparam A_IMADDR_H = 'h0D;
  localparam A_STATUS2 = 'h0E;
  localparam A_DOORBELL = 'h0F;
  localparam A_IMDATA = 'h10;
  localparam A_PIN_OD = 'h14;
  localparam A_OWNER = 'h18;
  localparam A_START_PC = 'h20;
  localparam A_PC = 'h22;
  localparam A_CSTAT = 'h24;
  localparam A_DMADDR = 'h28;
  localparam A_DMDATA = 'h29;

  localparam OWN_W = 2 * NUM_PINS;
  localparam CW4 = (FIFO_CW < 4) ? FIFO_CW : 4;

  wire [31:0] addr = {24'd0, r_addr};
  wire        wr = r_we;

  reg  [AW-1:0] imaddr;
  reg  [  23:0] stage;
  reg  [   3:0] irq_en;
  reg           imem_err;
  reg  [   4:0] st2;  // [0] ill [1] stk [3] tx_underrun [4] ctrl_irq ([2] is rx_overrun)

  wire wr_errclr = wr & (addr == A_ERR_CLR);
  wire wr_st2 = wr & (addr == A_STATUS2);
  wire im_commit = wr & (addr == A_IMDATA + 3);

  assign im_we           = im_commit & ~run;
  assign im_waddr        = imaddr;
  assign txf_wr          = wr & (addr == A_TXF);
  assign txf_wdata       = r_wdata;
  assign rxf_rd          = r_re & (addr == A_RXF);
  assign txf_clr_overrun = wr_errclr & r_wdata[4];
  assign rxf_clr_overrun = wr_errclr & r_wdata[5];
  assign owner_clr_err   = wr_errclr & r_wdata[6];
  assign restart         = wr & (addr == A_RESTART) & r_wdata[0];
  assign db_set          = wr & (addr == A_DOORBELL);
  assign rx_ovr_clr      = wr_st2 & r_wdata[2];
  assign dm_acc          = (wr | r_re) & (addr == A_DMDATA);
  assign dm_we           = wr & (addr == A_DMDATA);
  assign dm_wdata        = r_wdata;

  always @(*) im_wdata = {r_wdata, stage};

  wire [7:0] status2 = {3'd0, st2[4:3], rx_overrun, st2[1:0]};
  wire any_err = txf_overrun | rxf_overrun | owner_err | imem_err | (|status2[3:0]);
  assign irq = (irq_en[0] & ~rxf_empty) | (irq_en[1] & ~txf_full) | (irq_en[2] & any_err) |
               (irq_en[3] & st2[4]);

  wire [15:0] imaddr16 = {{(16 - AW) {1'b0}}, imaddr};
  wire [15:0] spc16 = {{(16 - AW) {1'b0}}, start_pc};
  wire [15:0] pc16 = {{(16 - AW) {1'b0}}, pc};
  wire [15:0] pin16;
  wire [15:0] od16;
  wire [31:0] own32;
  generate
    if (NUM_PINS >= 16) begin : g_p16
      assign pin16 = pin_in[15:0];
      assign od16  = pin_od[15:0];
      assign own32 = pin_owner[31:0];
    end else begin : g_plt
      assign pin16 = {{(16 - NUM_PINS) {1'b0}}, pin_in};
      assign od16  = {{(16 - NUM_PINS) {1'b0}}, pin_od};
      assign own32 = {{(32 - OWN_W) {1'b0}}, pin_owner};
    end
  endgenerate

  integer b, rb;

  always @(posedge clk) begin
    if (rst) begin
      run       <= 1'b0;
      start_pc  <= {AW{1'b0}};
      pin_owner <= {OWN_W{1'b0}};
      pin_od    <= {NUM_PINS{1'b0}};
      imaddr    <= {AW{1'b0}};
      stage     <= 24'd0;
      irq_en    <= 4'd0;
      imem_err  <= 1'b0;
      st2       <= 5'd0;
      dm_addr   <= 8'd0;
    end else begin
      if (wr) begin
        if (addr == A_RUN) run <= r_wdata[0];
        if (addr == A_IRQ_EN) irq_en <= r_wdata[3:0];
        if (addr == A_IMDATA + 0) stage[7:0] <= r_wdata;
        if (addr == A_IMDATA + 1) stage[15:8] <= r_wdata;
        if (addr == A_IMDATA + 2) stage[23:16] <= r_wdata;
        for (b = 0; b < NUM_PINS; b = b + 1) if (addr == A_PIN_OD + b / 8) pin_od[b] <= r_wdata[b%8];
        for (b = 0; b < OWN_W; b = b + 1) if (addr == A_OWNER + b / 8) pin_owner[b] <= r_wdata[b%8];
        for (b = 0; b < AW; b = b + 1) if (addr == A_START_PC + b / 8) start_pc[b] <= r_wdata[b%8];
      end

      // IMEM address: written by the host, or auto-incremented on commit.
      if (wr && addr == A_IMADDR) begin
        for (b = 0; b < AW && b < 8; b = b + 1) imaddr[b] <= r_wdata[b];
      end else if (wr && addr == A_IMADDR_H) begin
        for (b = 8; b < AW; b = b + 1) imaddr[b] <= r_wdata[b-8];
      end else if (im_we) imaddr <= imaddr + 1'b1;

      if (im_commit && run) imem_err <= 1'b1;
      else if (wr_errclr && r_wdata[7]) imem_err <= 1'b0;

      // STATUS2: sticky set wins over W1C
      st2[0] <= ill_err | (st2[0] & ~(wr_st2 & r_wdata[0]));
      st2[1] <= stk_err | (st2[1] & ~(wr_st2 & r_wdata[1]));
      st2[2] <= 1'b0;
      st2[3] <= tx_underrun | (st2[3] & ~(wr_st2 & r_wdata[3]));
      st2[4] <= ctrl_irq | (st2[4] & ~(wr_st2 & r_wdata[4]));

      if (wr && addr == A_DMADDR) dm_addr <= r_wdata;
      else if (dm_acc) dm_addr <= dm_addr + 1'b1;
    end
  end

  // ------------------------------------------------------------ readback
  always @(*) begin
    r_rdata = 8'd0;
    case (addr)
      A_ID: r_rdata = ID[7:0];
      A_VERSION: r_rdata = VERSION[7:0];
      A_RUN: r_rdata = {7'd0, run};
      A_STATUS:
      r_rdata = {
        imem_err, owner_err, rxf_overrun, txf_overrun, rxf_full, rxf_empty, txf_full, txf_empty
      };
      A_IRQ_EN: r_rdata = {4'd0, irq_en};
      A_RXF: r_rdata = rxf_rdata;
      A_IMADDR: r_rdata = imaddr16[7:0];
      A_IMADDR_H: r_rdata = imaddr16[15:8];
      A_PIN_IN: r_rdata = pin16[7:0];
      A_PIN_IN + 1: r_rdata = pin16[15:8];
      A_STATUS2: r_rdata = status2;
      A_PIN_OD: r_rdata = od16[7:0];
      A_PIN_OD + 1: r_rdata = od16[15:8];
      A_OWNER: r_rdata = own32[7:0];
      A_OWNER + 1: r_rdata = own32[15:8];
      A_OWNER + 2: r_rdata = own32[23:16];
      A_OWNER + 3: r_rdata = own32[31:24];
      A_START_PC: r_rdata = spc16[7:0];
      A_START_PC + 1: r_rdata = spc16[15:8];
      A_PC: r_rdata = pc16[7:0];
      A_PC + 1: r_rdata = pc16[15:8];
      A_CSTAT: r_rdata = {cstat_in[7:1], run};
      A_DMADDR: r_rdata = dm_addr;
      A_DMDATA: r_rdata = dm_rdata;
      default: ;
    endcase
    for (rb = 0; rb < CW4; rb = rb + 1) begin
      if (addr == A_FIFO_CNT) begin
        r_rdata[rb]   = txf_count[rb];
        r_rdata[4+rb] = rxf_count[rb];
      end
    end
  end

  wire _unused = &{1'b0, cstat_in[0], st2[2]};

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // * IMEM is written only by a commit to 0x13 with RUN = 0; a commit while
  //   running sets imem_err (sticky until ERR_CLR bit 7).
  // * Strobes and pulses only for their own address and access type.
  // * No write -> no configuration register changes; a write changes only
  //   the register at its address (RUN, PIN_OD, PIN_OWNER, START_PC, IRQ_EN).
  // * IMEM_ADDR / DMEM_ADDR auto-increment rules. STATUS2 stickiness (set
  //   wins over W1C). irq equation.
`ifdef PE_REGS_TOP
  initial assume (rst);
`endif
`ifdef PE_CHIP_FORMAL
  wire f_rst_seen = f_seen_i;  // one flag for the whole chip proof
`else
  reg f_rst_seen = 1'b0;
  always @(posedge clk) if (rst) f_rst_seen <= 1'b1;
`endif
  wire f_ok = f_rst_seen & ~rst;
  reg  f_past_ok = 1'b0;
  always @(posedge clk) f_past_ok <= f_ok;

  always @(*) begin
    if (f_ok) begin
      if (im_we) assert (r_we && addr == A_IMDATA + 3 && !run);
      assert (txf_wr == (r_we && addr == A_TXF));
      assert (rxf_rd == (r_re && addr == A_RXF));
      assert (restart == (r_we && addr == A_RESTART && r_wdata[0]));
      assert (db_set == (r_we && addr == A_DOORBELL));
      assert (dm_we == (r_we && addr == A_DMDATA));
      if (dm_we) assert (dm_acc);
      if (txf_clr_overrun || rxf_clr_overrun || owner_clr_err) assert (r_we && addr == A_ERR_CLR);
      if (rx_ovr_clr) assert (r_we && addr == A_STATUS2);
      assert (irq == ((irq_en[0] & ~rxf_empty) | (irq_en[1] & ~txf_full) | (irq_en[2] & any_err) |
                      (irq_en[3] & st2[4])));
      assert (!st2[2]);
`ifdef PE_REGS_TOP  // module-level covers only when this module is the proof top
      cover (im_we);
      cover (imem_err);
`endif
    end
  end

  always @(posedge clk) begin
    if (f_past_ok && f_ok) begin
      if (!$past(r_we)) begin
        assert (run == $past(run) && start_pc == $past(start_pc) && irq_en == $past(irq_en));
        assert (pin_owner == $past(pin_owner) && pin_od == $past(pin_od));
        assert (stage == $past(stage));
      end else begin
        if ($past(addr) == A_RUN) assert (run == $past(r_wdata[0]));
        else assert (run == $past(run));
        if ($past(addr) < A_OWNER || $past(addr) >= A_OWNER + 4) assert (pin_owner == $past(pin_owner));
        if ($past(addr) != A_PIN_OD && $past(addr) != A_PIN_OD + 1) assert (pin_od == $past(pin_od));
        if ($past(addr) != A_START_PC && $past(addr) != A_START_PC + 1)
          assert (start_pc == $past(start_pc));
        if ($past(addr) != A_IRQ_EN) assert (irq_en == $past(irq_en));
      end
      if (!($past(r_we) && ($past(addr) == A_IMADDR || $past(addr) == A_IMADDR_H))) begin
        if ($past(im_we)) assert (imaddr == $past(imaddr) + 1'b1);
        else assert (imaddr == $past(imaddr));
      end
      if ($past(r_we) && $past(addr) == A_DMADDR) assert (dm_addr == $past(r_wdata));
      else if ($past(dm_acc)) assert (dm_addr == $past(dm_addr) + 8'd1);
      else assert (dm_addr == $past(dm_addr));
      if ($past(im_commit && run)) assert (imem_err);
      else if ($past(wr_errclr && r_wdata[7])) assert (!imem_err);
      else assert (imem_err == $past(imem_err));
      if ($past(ill_err)) assert (st2[0]);
      else if ($past(wr_st2 && r_wdata[0])) assert (!st2[0]);
      else assert (st2[0] == $past(st2[0]));
      if ($past(ctrl_irq)) assert (st2[4]);
      else if ($past(wr_st2 && r_wdata[4])) assert (!st2[4]);
      else assert (st2[4] == $past(st2[4]));
    end
  end
`endif

endmodule
