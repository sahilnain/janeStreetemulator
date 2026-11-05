/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_crc: CRC unit, unit id 4 (docs/spec/blocks.md Section 5a). One 16-bit
 * register, top-aligned, so a CRC of width w < 16 uses POLY << (16-w) and
 * INIT << (16-w). The bit stream is the TX unit's or the RX unit's data bits
 * (after line decoding, without stuffed bits).
 *
 * Config (ucfg CRC, idx):
 *   0 CTRL [1:0] SRC (0 off, 1 TX, 2 RX) [2] APPEND [15:8] SKIP
 *   1 POLY [15:0]
 *   2 INIT [15:0]: writing loads the register, reloads the SKIP counter and
 *                  clears `sent`. Write CTRL and POLY first.
 * Read (urd CRC, idx): 0 the register, 1 [0] sent
 *
 * Each bit: if SKIP bits remain, count one down; else
 *   crc <- (crc << 1) ^ ((crc[15] ^ bit) ? POLY : 0).
 * APPEND: when the TX FIFO is empty at the end of a chained word, the TX unit
 * sends ~crc, 16 bits MSB-first, once (USB CRC16).
 * USB residuals: CRC16 0x800D; CRC5 (POLY 0x2800, INIT 0xF800) 0x6000.
 */

`default_nettype none

module pe_crc (
    input wire clk,
    input wire rst,
`ifdef PE_CHIP_FORMAL
    input wire f_seen_i,
`endif

    input  wire        cfg_we,
    input  wire [ 2:0] cfg_idx,
    input  wire [15:0] cfg_data,
    input  wire [ 2:0] rd_idx,
    output reg  [15:0] rd_data,

    input wire tx_v,
    input wire tx_bit,
    input wire rx_v,
    input wire rx_bit,
    input wire sent_set,

    output wire        app_ok,
    output wire [15:0] value
);

  reg [ 1:0] c_src;
  reg        c_app;
  reg [ 7:0] c_skip;
  reg [15:0] c_poly;
  reg [15:0] crc;
  reg [ 7:0] skip;
  reg        sent;

  wire v = (c_src == 2'd1) ? tx_v : (c_src == 2'd2) ? rx_v : 1'b0;
  wire b = (c_src == 2'd1) ? tx_bit : rx_bit;
  wire fb = crc[15] ^ b;
  wire [15:0] crc_nx = {crc[14:0], 1'b0} ^ (fb ? c_poly : 16'd0);
  wire load = cfg_we & (cfg_idx == 3'd2);

  assign app_ok = c_app & ~sent;
  assign value  = ~crc;

  always @(posedge clk) begin
    if (rst) begin
      c_src  <= 2'd0;
      c_app  <= 1'b0;
      c_skip <= 8'd0;
      c_poly <= 16'd0;
      crc    <= 16'd0;
      skip   <= 8'd0;
      sent   <= 1'b0;
    end else begin
      if (cfg_we && cfg_idx == 3'd0) {c_skip, c_app, c_src} <= {cfg_data[15:8], cfg_data[2:0]};
      if (cfg_we && cfg_idx == 3'd1) c_poly <= cfg_data;
      if (load) begin
        crc  <= cfg_data;
        skip <= c_skip;
        sent <= 1'b0;
      end else begin
        if (v) begin
          if (skip != 8'd0) skip <= skip - 1'b1;
          else crc <= crc_nx;
        end
        if (sent_set) sent <= 1'b1;
      end
    end
  end

  always @(*) begin
    case (rd_idx)
      3'd0: rd_data = crc;
      3'd1: rd_data = {15'd0, sent};
      default: rd_data = 16'd0;
    endcase
  end

  wire _unused = &{1'b0, cfg_data[7:3]};

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // A reference CRC register computed bit by bit gives the same value for any
  // POLY; a skipped bit changes nothing; sent is set once and cleared by INIT.
`ifdef PE_CRC_TOP
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

  always @(posedge clk) begin
    if (f_past_ok && f_ok) begin
      if ($past(load)) begin
        assert (crc == $past(cfg_data) && skip == $past(c_skip) && !sent);
      end else begin
        if ($past(v) && $past(skip) != 8'd0) begin
          assert (crc == $past(crc) && skip == $past(skip) - 8'd1);
        end else if ($past(v)) begin
          assert (skip == 8'd0);
          assert (crc[0] == ($past(fb) & $past(c_poly[0])));
          assert (crc == (($past(crc) << 1) ^ ($past(fb) ? $past(c_poly) : 16'd0)));
        end else begin
          assert (crc == $past(crc) && skip == $past(skip));
        end
        if ($past(sent_set)) assert (sent);
        else assert (sent == $past(sent));
      end
    end
  end
  always @(*) begin
    if (f_ok) begin
      assert (app_ok == (c_app && !sent));
      assert (value == ~crc);
`ifdef PE_CRC_TOP
      cover (crc == 16'h800D && !sent && c_poly == 16'h8005);
`endif
    end
  end
`endif

endmodule
