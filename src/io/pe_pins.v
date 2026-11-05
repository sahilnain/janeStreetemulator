/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_pins: pin fabric (docs/spec/blocks.md Section 7).
 *
 * - Each pin has an out reg and an oe reg. They hold their value until a
 *   port writes them. Reset: out = OUT_RESET (idle high), oe = 0.
 * - Owner map, 2 bits per pin: 0 controller, 1 TX unit, 2 RX unit,
 *   3 controller.
 * - Three write ports (out mask/value and oe mask/value each):
 *     CTRL  controller: allowed if the pin is controller-owned, or its
 *           owner unit is idle (tx_busy / rx_busy = 0)
 *     TXP   TX unit and the ETX drive: allowed on TX-owned pins
 *     RXP   RX unit: allowed on RX-owned pins
 *   A write that is not allowed is dropped for that pin and sets the sticky
 *   owner_err. Allowed writers never collide (CTRL needs the owner idle);
 *   the fixed priority TXP > RXP > CTRL is only a fallback.
 * - Open-drain mode (pin_od[i] = 1): out = 0 drives low, out = 1 releases;
 *   the oe reg is ignored.
 * - Inputs go through a 2-FF synchroniser (pin_in).
 */

`default_nettype none

module pe_pins #(
    parameter NUM_PINS  = 8,
    parameter OUT_RESET = {NUM_PINS{1'b1}}  // idle-high, safe for UART/I2C
) (
    input wire clk,
    input wire rst,
`ifdef PE_CHIP_FORMAL
    input wire f_seen_i,
`endif

    input wire [NUM_PINS-1:0] c_wm,
    input wire [NUM_PINS-1:0] c_wv,
    input wire [NUM_PINS-1:0] c_om,
    input wire [NUM_PINS-1:0] c_ov,
    input wire [NUM_PINS-1:0] t_wm,
    input wire [NUM_PINS-1:0] t_wv,
    input wire [NUM_PINS-1:0] t_om,
    input wire [NUM_PINS-1:0] t_ov,
    input wire [NUM_PINS-1:0] r_wm,
    input wire [NUM_PINS-1:0] r_wv,

    input wire tx_busy,
    input wire rx_busy,

    output wire [NUM_PINS-1:0] pin_in,

    input  wire [2*NUM_PINS-1:0] pin_owner,
    input  wire [  NUM_PINS-1:0] pin_od,
    output reg                   owner_err,
    input  wire                  clr_err,

    output wire [NUM_PINS-1:0] pad_out,
    output wire [NUM_PINS-1:0] pad_oe,
    input  wire [NUM_PINS-1:0] pad_in
`ifdef FORMAL
    ,  // formal-only: lets the whole-chip proof see the pin registers
    output wire [NUM_PINS-1:0] f_out_r,
    output wire [NUM_PINS-1:0] f_oe_r
`endif
);

  reg  [NUM_PINS-1:0] out_r;
  reg  [NUM_PINS-1:0] oe_r;
  wire [NUM_PINS-1:0] own_t, own_r, ok_c;

  genvar i;
  generate
    for (i = 0; i < NUM_PINS; i = i + 1) begin : g_own
      wire [1:0] o = pin_owner[2*i+:2];
      assign own_t[i] = (o == 2'd1);
      assign own_r[i] = (o == 2'd2);
      assign ok_c[i]  = (o == 2'd0) | (o == 2'd3) | (own_t[i] & ~tx_busy) | (own_r[i] & ~rx_busy);
    end
  endgenerate

  wire [NUM_PINS-1:0] t_wh = t_wm & own_t;
  wire [NUM_PINS-1:0] t_oh = t_om & own_t;
  wire [NUM_PINS-1:0] r_wh = r_wm & own_r;
  wire [NUM_PINS-1:0] c_wh = c_wm & ok_c & ~t_wh & ~r_wh;
  wire [NUM_PINS-1:0] c_oh = c_om & ok_c & ~t_oh;

  wire [NUM_PINS-1:0] wr_hit = t_wh | r_wh | c_wh;
  wire [NUM_PINS-1:0] wr_val = (t_wv & t_wh) | (r_wv & r_wh) | (c_wv & c_wh);
  wire [NUM_PINS-1:0] oe_hit = t_oh | c_oh;
  wire [NUM_PINS-1:0] oe_val = (t_ov & t_oh) | (c_ov & c_oh);

  wire denied = |(t_wm & ~own_t) | |(t_om & ~own_t) | |(r_wm & ~own_r) |
                |(c_wm & ~ok_c) | |(c_om & ~ok_c);

  always @(posedge clk) begin
    if (rst) begin
      out_r     <= OUT_RESET[NUM_PINS-1:0];
      oe_r      <= {NUM_PINS{1'b0}};
      owner_err <= 1'b0;
    end else begin
      out_r <= (out_r & ~wr_hit) | wr_val;
      oe_r  <= (oe_r & ~oe_hit) | oe_val;
      if (denied) owner_err <= 1'b1;
      else if (clr_err) owner_err <= 1'b0;
    end
  end

  assign pad_out = out_r & ~pin_od;
  assign pad_oe  = (pin_od & ~out_r) | (~pin_od & oe_r);

  pe_sync #(
      .W(NUM_PINS)
  ) u_sync (
      .clk(clk),
      .rst(rst),
      .d  (pad_in),
      .q  (pin_in)
  );

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // * Pad mapping: open-drain pins never drive high; push-pull pins drive
  //   out / oe directly.
  // * Ownership: a pin's out / oe only changes on a hit of an allowed port, and
  //   then takes the written value. TXP only reaches TX-owned pins, RXP only
  //   RX-owned pins, CTRL only controller-owned pins or those of an idle unit.
  // * At most one port hits a pin in a cycle.
  // * owner_err: set by any denied bit, cleared by clr_err, otherwise stable.
  // * Reset: out = OUT_RESET, oe = 0, no error.
`ifdef PE_PINS_TOP
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

  assign f_out_r = out_r;
  assign f_oe_r  = oe_r;

  always @(*) begin
    if (f_ok) begin
      assert ((pad_out & pin_od) == {NUM_PINS{1'b0}});
      assert ((pad_oe & pin_od) == (~out_r & pin_od));
      assert ((pad_oe & ~pin_od) == (oe_r & ~pin_od));
      assert ((pad_out & ~pin_od) == (out_r & ~pin_od));
      assert ((t_wh & ~own_t) == {NUM_PINS{1'b0}} && (t_oh & ~own_t) == {NUM_PINS{1'b0}});
      assert ((r_wh & ~own_r) == {NUM_PINS{1'b0}});
      assert ((c_wh & ~ok_c) == {NUM_PINS{1'b0}} && (c_oh & ~ok_c) == {NUM_PINS{1'b0}});
      assert ((t_wh & r_wh) == {NUM_PINS{1'b0}});
      assert ((c_wh & (t_wh | r_wh)) == {NUM_PINS{1'b0}});
      assert ((c_oh & t_oh) == {NUM_PINS{1'b0}});
      assert ((own_t & own_r) == {NUM_PINS{1'b0}});
      if (tx_busy) assert ((c_wh & own_t) == {NUM_PINS{1'b0}} && (c_oh & own_t) == {NUM_PINS{1'b0}});
      if (rx_busy) assert ((c_wh & own_r) == {NUM_PINS{1'b0}} && (c_oh & own_r) == {NUM_PINS{1'b0}});
`ifdef PE_PINS_TOP  // module-level covers only when this module is the proof top
      cover (owner_err);
`endif
    end
  end

  always @(posedge clk) begin
    if (f_past_ok && f_ok) begin
      assert (((out_r ^ $past(out_r)) & ~$past(wr_hit)) == {NUM_PINS{1'b0}});
      assert ((out_r & $past(wr_hit)) == ($past(wr_val) & $past(wr_hit)));
      assert (((oe_r ^ $past(oe_r)) & ~$past(oe_hit)) == {NUM_PINS{1'b0}});
      assert ((oe_r & $past(oe_hit)) == ($past(oe_val) & $past(oe_hit)));
      if ($past(denied)) assert (owner_err);
      else if ($past(clr_err)) assert (!owner_err);
      else assert (owner_err == $past(owner_err));
    end
    if (f_rst_seen && $past(rst)) begin
      assert (out_r == OUT_RESET[NUM_PINS-1:0]);
      assert (oe_r == {NUM_PINS{1'b0}});
      assert (!owner_err);
    end
  end
`endif

endmodule
