/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * tt_um_sahil_protoemu: Protocol Emulator v2 top level (Tiny Tapeout wrapper).
 *
 * Pin map (unchanged from v1)
 *   ui[3:0]  host data in        uo[3:0]  host data out
 *   ui[5:4]  host op             uo[4]    host ack (toggle)
 *   ui[6]    host strobe (toggle) uo[5]   IRQ
 *   ui[7]    reserved            uo[7:6]  reserved (0)
 *   uio[7:0] protocol pins P0-P7 (direction set per pin by firmware / open-drain)
 *
 * Architecture (docs/spec/blocks.md):
 *   host bus -> pe_host_if -> pe_regs -> {pe_imem, FIFOs, pe_dmem, status}
 *   pe_imem -> pe_ctrl -> {pe_txu, pe_rxu, pe_det ETX/ERX} -> pe_pins -> uio
 *
 * Glue in this file:
 *   - unit start: from `ustart` or the unit's own detector (START), only
 *     when the unit is idle. One TX FIFO pop serves every unit that starts
 *     in the cycle and needs a byte (TX DEN, RX drive mode), unless the
 *     byte comes from `ustart`'s op2. A detector start with no byte is
 *     dropped and sets STATUS2.tx_underrun.
 *   - FIFO arbitration: units and the host win; the controller is granted
 *     only when they do not use the port (it blocks and retries).
 *   - RX busy includes the ERX after-packet step (stop-bit check).
 *   - CHAIN: when a TX word ends, the next word comes from the TX FIFO or, if
 *     that is empty and the CRC unit has APPEND set, is the CRC.
 *   - A detector with STOP set stops its unit in the fire cycle.
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
  localparam IMEM_DEPTH = 160;  // USB device firmware is about 150 words; revisit in the area pass
  localparam IMEM_IMPL = 0;  // 0 = flops, 1 = latches
  localparam DMEM_DEPTH = 256;
  localparam FIFO_DEPTH = 8;
  localparam STACK_DEPTH = 4;
  localparam NUM_PINS = 8;  // = uio width

  // Protocol support (docs/spec/blocks.md Section 13). UART is always built. Each macro defaults to
  // 1; define it as 0 (for example -DSUPPORT_USB=0) to leave that protocol's features out.
`ifndef SUPPORT_SPI
`define SUPPORT_SPI 1
`endif
`ifndef SUPPORT_I2C
`define SUPPORT_I2C 1
`endif
`ifndef SUPPORT_USB
`define SUPPORT_USB 1
`endif
  localparam SUPPORT_SPI = `SUPPORT_SPI;
  localparam SUPPORT_I2C = `SUPPORT_I2C;
  localparam SUPPORT_USB = `SUPPORT_USB;

  localparam AW = (IMEM_DEPTH > 1) ? $clog2(IMEM_DEPTH) : 1;
  localparam FIFO_CW = $clog2(FIFO_DEPTH + 1);

`ifdef FORMAL
  reg f_rst_seen = 1'b0;  // reset has been seen (shared with the sub-modules in the chip proof)
`endif
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

  // ------------------------------------------------------- signals
  wire im_we;
  wire [AW-1:0] im_waddr, im_raddr;
  wire [31:0] im_wdata, im_rdata;

  wire h_txf_wr, h_rxf_rd, txf_full, txf_empty, txf_overrun, txf_clr;
  wire rxf_full, rxf_empty, rxf_overrun, rxf_clr;
  wire [7:0] h_txf_wdata, txf_rdata, rxf_rdata;
  wire [FIFO_CW-1:0] txf_count, rxf_count;

  wire run, restart, db_set;
  wire [AW-1:0] start_pc, pc;
  wire ill_err, stk_err, host_irq, tx_underrun;
  wire rx_overrun, rx_ovr_clr;

  wire [2*NUM_PINS-1:0] pin_owner;
  wire [NUM_PINS-1:0] pin_od, pin_filt, pin_in;
  wire owner_err, owner_clr;

  wire h_dm_acc, h_dm_we;
  wire [7:0] h_dm_addr, h_dm_wdata, dm_rdata;

  // controller <-> blocks
  wire [NUM_PINS-1:0] c_wm, c_wv, c_om, c_ov;
  wire c_txf_rd, c_rxf_rd, c_txf_wr, c_rxf_wr;
  wire [7:0] c_fdata;
  wire [7:0] c_dm_addr, c_dm_wdata;
  wire c_dm_we, c_dm_gnt;
  wire cfg_we;
  wire [2:0] cfg_unit, cfg_idx, rd_unit, rd_idx;
  wire [15:0] cfg_data;
  reg [15:0] rd_data;
  wire rd_strobe;
  wire [1:0] start_mask, stop_mask;
  wire start_imm;
  wire [15:0] c_start_data;
  wire det_sel, det_clr, det_arm, det_disarm;
  wire z, c, to, db, blocked;

  // units
  wire tx_busy, tx_done, tx_stick, tx_dtick, tx_den, tx_chain, tx_bit_v, tx_bit;
  wire [NUM_PINS-1:0] tx_d2m;
  wire [3:0] tx_dpin;
  wire [NUM_PINS-1:0] tx_wm, tx_wv;
  wire [15:0] tx_rd;
  wire rx_busy_core, rx_done, rx_push, rxrdy, rx_need, rx_bit_v, rx_bit;
  wire etx_stop, erx_stop, rx_chain_on;
  wire crc_app_ok;
  wire [15:0] crc_rd, crc_val;
  wire [7:0] rx_push_data;
  wire [3:0] rx_ipin;
  wire [NUM_PINS-1:0] rx_wm, rx_wv;
  wire [15:0] rx_rd;
  wire etx_req, etx_pend, etx_end, etx_flag, etx_ferr;
  wire erx_req, erx_pend, erx_end, erx_flag, ferr;
  wire [NUM_PINS-1:0] etx_wm, etx_wv, etx_om, etx_ov, erx_wm, erx_wv, erx_om, erx_ov;
  wire [15:0] etx_rd, erx_rd;

  wire rx_busy = rx_busy_core | erx_pend;

  // ------------------------------------------------------ unit starts
  wire use_imm = (|start_mask) & start_imm;
  wire byte_avail = use_imm | ~txf_empty;
  // `ustop` wins over a detector start in the same cycle; RESTART wins over any start.
  wire etx_raw = etx_req & ~tx_busy;
  wire erx_raw = erx_req & ~rx_busy;
  wire etx_try = etx_raw & ~stop_mask[0] & ~restart;
  wire erx_try = erx_raw & ~stop_mask[1] & ~restart;
  wire tx_go = (start_mask[0] & ~restart) | (etx_try & (~tx_den | byte_avail));
  wire rx_go = (start_mask[1] & ~restart) | (erx_try & (~rx_need | byte_avail));
  assign tx_underrun = (etx_try & tx_den & ~byte_avail) | (erx_try & rx_need & ~byte_avail);
  wire u_pop = ((tx_go & tx_den) | (rx_go & rx_need)) & ~use_imm;
  // 0 on an empty FIFO: a start that needs no byte must not depend on stale FIFO memory
  wire [15:0] u_data = use_imm ? c_start_data : txf_empty ? 16'd0 : {8'd0, txf_rdata};

  // CHAIN: the next TX word is a FIFO byte, or else the CRC.
  wire tx_next_crc = txf_empty & crc_app_ok;
  wire tx_next_ok = ~txf_empty | crc_app_ok;
  wire [15:0] tx_next_data = txf_empty ? crc_val : {8'd0, txf_rdata};
  wire chain_pop_raw = tx_chain & ~tx_next_crc;
  wire chain_pop = chain_pop_raw & ~stop_mask[0] & ~etx_stop & ~restart;

  // pops started by a detector alone (independent of the controller's outputs)
  wire det_pop = (((etx_raw & tx_den) | (erx_raw & rx_need)) & ~txf_empty) | chain_pop_raw;

  // ---------------------------------------------------------- FIFOs
  wire txf_wr = h_txf_wr | c_txf_wr;
  wire [7:0] txf_wdata = h_txf_wr ? h_txf_wdata : c_fdata;
  wire txf_rd = u_pop | chain_pop | c_txf_rd;
  wire rxf_wr = rx_push | c_rxf_wr;
  wire [7:0] rxf_wdata = rx_push ? rx_push_data : c_fdata;
  wire rxf_rd = h_rxf_rd | c_rxf_rd;

  pe_fifo #(
      .W    (8),
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
      .W    (8),
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

  // ------------------------------------------------------ registers
  pe_regs #(
      .SUPPORT_I2C(SUPPORT_I2C),
      .IMEM_DEPTH(IMEM_DEPTH),
      .NUM_PINS  (NUM_PINS),
      .FIFO_CW   (FIFO_CW)
  ) u_regs (
      .clk            (clk),
      .rst            (rst),
`ifdef PE_CHIP_FORMAL
      .f_seen_i  (f_rst_seen),
`endif
      .r_addr         (r_addr),
      .r_wdata        (r_wdata),
      .r_we           (r_we),
      .r_re           (r_re),
      .r_rdata        (r_rdata),
      .irq            (irq),
      .im_we          (im_we),
      .im_waddr       (im_waddr),
      .im_wdata       (im_wdata),
      .txf_wr         (h_txf_wr),
      .txf_wdata      (h_txf_wdata),
      .txf_full       (txf_full),
      .txf_empty      (txf_empty),
      .txf_count      (txf_count),
      .txf_overrun    (txf_overrun),
      .txf_clr_overrun(txf_clr),
      .rxf_rd         (h_rxf_rd),
      .rxf_rdata      (rxf_rdata),
      .rxf_full       (rxf_full),
      .rxf_empty      (rxf_empty),
      .rxf_count      (rxf_count),
      .rxf_overrun    (rxf_overrun),
      .rxf_clr_overrun(rxf_clr),
      .run            (run),
      .restart        (restart),
      .start_pc       (start_pc),
      .db_set         (db_set),
      .pc             (pc),
      .cstat_in       ({db, erx_flag, etx_flag, to, c, z, blocked, 1'b0}),
      .ill_err        (ill_err),
      .stk_err        (stk_err),
      .tx_underrun    (tx_underrun),
      .ctrl_irq       (host_irq),
      .rx_overrun     (rx_overrun),
      .rx_ovr_clr     (rx_ovr_clr),
      .pin_owner      (pin_owner),
      .pin_od         (pin_od),
      .pin_filt       (pin_filt),
      .pin_in         (pin_in),
      .owner_err      (owner_err),
      .owner_clr_err  (owner_clr),
      .dm_acc         (h_dm_acc),
      .dm_we          (h_dm_we),
      .dm_addr        (h_dm_addr),
      .dm_wdata       (h_dm_wdata),
      .dm_rdata       (dm_rdata)
  );

  // ------------------------------------------------- memories
  pe_imem #(
      .IW   (32),
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

  pe_dmem #(
      .DEPTH(DMEM_DEPTH)
  ) u_dmem (
      .clk    (clk),
      .h_acc  (h_dm_acc),
      .h_we   (h_dm_we),
      .h_addr (h_dm_addr),
      .h_wdata(h_dm_wdata),
      .c_we   (c_dm_we),
      .c_addr (c_dm_addr),
      .c_wdata(c_dm_wdata),
      .c_gnt  (c_dm_gnt),
      .rdata  (dm_rdata)
  );

  // ---------------------------------------------------- controller
  pe_ctrl #(
      .IMEM_DEPTH (IMEM_DEPTH),
      .NUM_PINS   (NUM_PINS),
      .STACK_DEPTH(STACK_DEPTH)
  ) u_ctrl (
      .clk       (clk),
      .rst       (rst),
`ifdef PE_CHIP_FORMAL
      .f_seen_i  (f_rst_seen),
`endif
      .run       (run),
      .restart   (restart),
      .start_pc  (start_pc),
      .db_set    (db_set),
      .im_raddr  (im_raddr),
      .im_rdata  (im_rdata),
      .pw_mask   (c_wm),
      .pw_val    (c_wv),
      .po_mask   (c_om),
      .po_val    (c_ov),
      .pin_in    (pin_in),
      .txf_empty (txf_empty),
      .txf_full  (txf_full),
      .txf_rdata (txf_rdata),
      .rxf_empty (rxf_empty),
      .rxf_full  (rxf_full),
      .rxf_rdata (rxf_rdata),
      .txf_rd_gnt(~det_pop),
      .txf_wr_gnt(~h_txf_wr),
      .rxf_rd_gnt(~h_rxf_rd),
      .rxf_wr_gnt(~rx_push),
      .txf_rd    (c_txf_rd),
      .rxf_rd    (c_rxf_rd),
      .txf_wr    (c_txf_wr),
      .rxf_wr    (c_rxf_wr),
      .f_wdata   (c_fdata),
      .dm_addr   (c_dm_addr),
      .dm_rdata  (dm_rdata),
      .dm_we     (c_dm_we),
      .dm_wdata  (c_dm_wdata),
      .dm_gnt    (c_dm_gnt),
      .cfg_we    (cfg_we),
      .cfg_unit  (cfg_unit),
      .cfg_idx   (cfg_idx),
      .cfg_data  (cfg_data),
      .rd_unit   (rd_unit),
      .rd_idx    (rd_idx),
      .rd_data   (rd_data),
      .rd_strobe (rd_strobe),
      .start_mask(start_mask),
      .start_imm (start_imm),
      .start_data(c_start_data),
      .stop_mask (stop_mask),
      .tx_busy   (tx_busy),
      .rx_busy   (rx_busy),
      .tx_need   (tx_den),
      .rx_need   (rx_need),
      .det_sel   (det_sel),
      .det_clr   (det_clr),
      .det_arm   (det_arm),
      .det_disarm(det_disarm),
      .etx_flag  (etx_flag),
      .erx_flag  (erx_flag),
      .ferr      (ferr),
      .rxrdy     (rxrdy),
      .host_irq  (host_irq),
      .ill_err   (ill_err),
      .stk_err   (stk_err),
      .pc        (pc),
      .z_o       (z),
      .c_o       (c),
      .to_o      (to),
      .db_o      (db),
      .blocked   (blocked)
  );

  always @(*) begin
    case (rd_unit)
      3'd0: rd_data = tx_rd;
      3'd1: rd_data = rx_rd;
      3'd2: rd_data = (rd_idx == 3'd0) ? etx_rd : 16'd0;
      3'd3: rd_data = (rd_idx == 3'd0) ? erx_rd : 16'd0;
      3'd4: rd_data = crc_rd;
      default: rd_data = 16'd0;
    endcase
  end

  // ---------------------------------------------------------- units
  pe_txu #(
      .SUPPORT_SPI(SUPPORT_SPI),
      .SUPPORT_I2C(SUPPORT_I2C),
      .SUPPORT_USB(SUPPORT_USB),
      .NUM_PINS(NUM_PINS)
  ) u_txu (
      .clk       (clk),
      .rst       (rst),
`ifdef PE_CHIP_FORMAL
      .f_seen_i  (f_rst_seen),
`endif
      .cfg_we    (cfg_we && cfg_unit == 3'd0),
      .cfg_idx   (cfg_idx),
      .cfg_data  (cfg_data),
      .rd_idx    (rd_idx),
      .rd_data   (tx_rd),
      .start     (tx_go),
      .start_data(u_data),
      .stop      (stop_mask[0] | etx_stop | restart),
      .next_ok   (tx_next_ok),
      .next_crc  (tx_next_crc),
      .next_data (tx_next_data),
      .chain     (tx_chain),
      .bit_v     (tx_bit_v),
      .bit_o     (tx_bit),
      .pin_in    (pin_in),
      .busy      (tx_busy),
      .done      (tx_done),
      .stick     (tx_stick),
      .dtick     (tx_dtick),
      .pw_mask   (tx_wm),
      .pw_val    (tx_wv),
      .den       (tx_den),
      .dpin      (tx_dpin),
      .d2_mask   (tx_d2m)
  );

  pe_rxu #(
      .SUPPORT_SPI(SUPPORT_SPI),
      .SUPPORT_I2C(SUPPORT_I2C),
      .SUPPORT_USB(SUPPORT_USB),
      .NUM_PINS(NUM_PINS)
  ) u_rxu (
      .clk       (clk),
      .rst       (rst),
`ifdef PE_CHIP_FORMAL
      .f_seen_i  (f_rst_seen),
`endif
      .cfg_we    (cfg_we && cfg_unit == 3'd1),
      .cfg_idx   (cfg_idx),
      .cfg_data  (cfg_data),
      .rd_idx    (rd_idx),
      .rd_data   (rx_rd),
      .rd_strobe (rd_strobe && rd_unit == 3'd1),
      .start     (rx_go),
      .start_data(u_data),
      .stop      (stop_mask[1] | erx_stop | restart),
      .restart   (restart),
      .pin_in    (pin_in),
      .tx_stick  (tx_stick),
      .tx_dtick  (tx_dtick),
      .after_end (erx_end),
      .ferr      (ferr),
      .clr_ovr   (rx_ovr_clr),
      .busy      (rx_busy_core),
      .done      (rx_done),
      .push      (rx_push),
      .push_data (rx_push_data),
      .rxrdy     (rxrdy),
      .overrun   (rx_overrun),
      .pw_mask   (rx_wm),
      .pw_val    (rx_wv),
      .need      (rx_need),
      .chain_on  (rx_chain_on),
      .ipin      (rx_ipin),
      .bit_v     (rx_bit_v),
      .bit_o     (rx_bit)
  );

  pe_det #(
      .KIND    (0),
      .SUPPORT_I2C(SUPPORT_I2C),
      .SUPPORT_USB(SUPPORT_USB),
      .NUM_PINS(NUM_PINS)
  ) u_etx (
      .clk       (clk),
      .rst       (rst),
`ifdef PE_CHIP_FORMAL
      .f_seen_i  (f_rst_seen),
`endif
      .cfg_we    (cfg_we && cfg_unit == 3'd2),
      .cfg_idx   (cfg_idx),
      .cfg_data  (cfg_data),
      .rd_data   (etx_rd),
      .clr       (det_clr & ~det_sel),
      .arm       (det_arm & ~det_sel),
      .disarm    (det_disarm & ~det_sel),
      .restart   (restart),
      .pin_in    (pin_in),
      .unit_busy (tx_busy),
      .unit_done (tx_done),
      .unit_stop (stop_mask[0] | etx_stop),
      .unit_chain(1'b0),
      .start_ack (etx_req & tx_go),
      .act_pin   (tx_dpin),
      .act2_mask (tx_d2m),
      .start_req (etx_req),
      .stop_req  (etx_stop),
      .after_pend(etx_pend),
      .after_end (etx_end),
      .flag      (etx_flag),
      .ferr      (etx_ferr),
      .pw_mask   (etx_wm),
      .pw_val    (etx_wv),
      .po_mask   (etx_om),
      .po_val    (etx_ov)
  );

  pe_det #(
      .KIND    (1),
      .SUPPORT_I2C(SUPPORT_I2C),
      .SUPPORT_USB(SUPPORT_USB),
      .NUM_PINS(NUM_PINS)
  ) u_erx (
      .clk       (clk),
      .rst       (rst),
`ifdef PE_CHIP_FORMAL
      .f_seen_i  (f_rst_seen),
`endif
      .cfg_we    (cfg_we && cfg_unit == 3'd3),
      .cfg_idx   (cfg_idx),
      .cfg_data  (cfg_data),
      .rd_data   (erx_rd),
      .clr       (det_clr & det_sel),
      .arm       (det_arm & det_sel),
      .disarm    (det_disarm & det_sel),
      .restart   (restart),
      .pin_in    (pin_in),
      .unit_busy (rx_busy_core),
      .unit_done (rx_done),
      .unit_stop (stop_mask[1] | erx_stop),
      .unit_chain(rx_chain_on),
      .start_ack (erx_req & rx_go),
      .act_pin   (rx_ipin),
      .act2_mask ({NUM_PINS{1'b0}}),
      .start_req (erx_req),
      .stop_req  (erx_stop),
      .after_pend(erx_pend),
      .after_end (erx_end),
      .flag      (erx_flag),
      .ferr      (ferr),
      .pw_mask   (erx_wm),
      .pw_val    (erx_wv),
      .po_mask   (erx_om),
      .po_val    (erx_ov)
  );

  pe_crc #(
      .SUPPORT_USB(SUPPORT_USB)
  ) u_crc (
      .clk      (clk),
      .rst      (rst),
`ifdef PE_CHIP_FORMAL
      .f_seen_i (f_rst_seen),
`endif
      .cfg_we   (cfg_we && cfg_unit == 3'd4),
      .cfg_idx  (cfg_idx),
      .cfg_data (cfg_data),
      .rd_idx   (rd_idx),
      .rd_data  (crc_rd),
      .tx_v     (tx_bit_v),
      .tx_bit   (tx_bit),
      .rx_v     (rx_bit_v),
      .rx_bit   (rx_bit),
      .sent_set (tx_chain & tx_next_crc),
      .app_ok   (crc_app_ok),
      .value    (crc_val)
  );

  // ------------------------------------------------------- pin fabric
`ifdef FORMAL
  wire [NUM_PINS-1:0] f_pins_out, f_pins_oe;
`endif
  pe_pins #(
      .NUM_PINS(NUM_PINS)
  ) u_pins (
      .clk      (clk),
      .rst      (rst),
`ifdef PE_CHIP_FORMAL
      .f_seen_i  (f_rst_seen),
`endif
      .c_wm     (c_wm),
      .c_wv     (c_wv),
      .c_om     (c_om),
      .c_ov     (c_ov),
      .t_wm     (tx_wm | etx_wm),
      .t_wv     ((tx_wv & tx_wm) | (etx_wv & etx_wm)),
      .t_om     (etx_om),
      .t_ov     (etx_ov),
      .r_wm     (rx_wm),
      .r_wv     (rx_wv),
      .tx_busy  (tx_busy),
      .rx_busy  (rx_busy),
      .pin_in   (pin_in),
      .pin_owner(pin_owner),
      .pin_od   (pin_od),
      .pin_filt (pin_filt),
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

  wire _unused = &{ena, ui_in[7], 1'b0, etx_pend, etx_end, etx_ferr, erx_wm, erx_wv, erx_om, erx_ov,
                   rx_wv & ~rx_wm};

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // Whole-chip proof. Every sub-module's properties are included; their input
  // contracts (no start while busy, done only while busy, ...) are ASSERTED
  // here. Only the host-bus protocol in pe_host_if stays an assumption,
  // because those inputs are chip pins. The other inputs are free.
  // Chip-level properties, unbounded (k-induction):
  // * uo_out[7:6] (reserved) are always 0.
  // * IMEM is never written while the controller runs.
  // * RESTART: both units idle and both detectors disarmed in the next cycle.
  // * `ustop` of an idle unit, or a stop with a detector firing, leaves it idle.
  // * The FIFO is never popped by a unit start when it is empty; the TX FIFO
  //   and RX FIFO ports have at most one user per cycle.
  // * Until the controller has run after reset: no protocol pin drives, the
  //   units are idle and the detectors disarmed.
  // Covers: from reset, driving ONLY the host-bus pins, the controller can be
  // started and a protocol pin can be driven.
  always @(*) assume (ena);
  reg [1:0] f_cyc = 2'd0;
  always @(posedge clk) if (f_cyc != 2'd3) f_cyc <= f_cyc + 1'b1;
  always @(*) if (f_cyc < 2'd3) assume (!rst_n);

  always @(posedge clk) if (rst) f_rst_seen <= 1'b1;
  wire f_ok = f_rst_seen & ~rst;
  reg  f_past_ok = 1'b0;
  always @(posedge clk) f_past_ok <= f_ok;

  reg f_ever_run;
  always @(posedge clk) begin
    if (rst) f_ever_run <= 1'b0;
    else if (run) f_ever_run <= 1'b1;
  end

  always @(*) begin
    assert (uo_out[7:6] == 2'b00);
    if (f_ok) begin
      if (im_we) assert (!run);
      if (u_pop) assert (!txf_empty);
      if (chain_pop) assert (!txf_empty && !c_txf_rd);
      if (tx_chain) assert (tx_busy);
      if (u_pop) assert (!c_txf_rd);
      if (h_txf_wr) assert (!c_txf_wr);
      if (rx_push) assert (!c_rxf_wr);
      if (h_rxf_rd) assert (!c_rxf_rd);
      if (tx_go && restart) assert (1'b0);
      if (rx_go && restart) assert (1'b0);
      if (tx_go) assert (!tx_busy);
      if (rx_go) assert (!rx_busy);
      if (!f_ever_run) begin
        assert (f_pins_out == {NUM_PINS{1'b1}} && f_pins_oe == {NUM_PINS{1'b0}});
        assert (uio_oe == 8'h00);
        assert (!tx_busy && !rx_busy);
        assert (etx_rd[2:0] == 3'd0 && erx_rd[2:0] == 3'd0);
      end
      cover (run);
      cover (uio_oe != 8'h00);
    end
  end

  always @(posedge clk) begin
    if (f_past_ok && f_ok && $past(restart)) begin
      assert (!tx_busy && !rx_busy_core && !erx_pend);
      assert (etx_rd[2:0] == 3'd0 && erx_rd[3:0] == 4'd0);
    end
  end
`endif

endmodule
