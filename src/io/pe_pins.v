/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_pins: pin fabric.
 *
 * - Each pin has an out reg and an oe reg. They hold their value until an
 *   instruction writes them.
 * - Static owner map: pin i can only be written by context pin_owner[i].
 *   Writes to pins the context does not own are ignored and set the sticky
 *   owner_err.
 * - Open-drain mode (pin_od[i]=1): out=0 drives low, out=1 releases (oe=0).
 *   The oe reg is ignored in this mode.
 * - Inputs go through a 2-FF synchroniser.
 *
 * Writes use a mask/value port, which covers `set` (1 pin), `out pin,n`
 * (n consecutive pins) and a future side-set field. Only one context
 * executes per cycle, so one port is enough.
 */

`default_nettype none

module pe_pins #(
    parameter NUM_PINS  = 8,
    parameter NUM_CTX   = 2,
    parameter OUT_RESET = {NUM_PINS{1'b1}},  // idle-high, safe for UART/I2C
    // derived, do not override
    parameter CTX_W     = (NUM_CTX > 1) ? $clog2(NUM_CTX) : 1
) (
    input  wire                      clk,
    input  wire                      rst,
    // write port from pe_core
    input  wire                      wr_en,
    input  wire [         CTX_W-1:0] wr_ctx,
    input  wire [      NUM_PINS-1:0] wr_mask,
    input  wire [      NUM_PINS-1:0] wr_val,
    input  wire                      oe_en,
    input  wire [      NUM_PINS-1:0] oe_mask,
    input  wire [      NUM_PINS-1:0] oe_val,
    // synchronised inputs to pe_core / pe_regs
    output wire [      NUM_PINS-1:0] pin_in,
    // configuration from pe_regs
    input  wire [NUM_PINS*CTX_W-1:0] pin_owner,
    input  wire [      NUM_PINS-1:0] pin_od,
    output reg                       owner_err,
    input  wire                      clr_err,
    // pads
    output wire [      NUM_PINS-1:0] pad_out,
    output wire [      NUM_PINS-1:0] pad_oe,
    input  wire [      NUM_PINS-1:0] pad_in
`ifdef FORMAL
    ,  // formal-only: lets the whole-chip proof see the pin registers
    output wire [      NUM_PINS-1:0] f_out_r,
    output wire [      NUM_PINS-1:0] f_oe_r
`endif
);

  reg  [NUM_PINS-1:0] out_r;
  reg  [NUM_PINS-1:0] oe_r;
  wire [NUM_PINS-1:0] own;

  genvar i;
  generate
    for (i = 0; i < NUM_PINS; i = i + 1) begin : g_own
      assign own[i] = (pin_owner[i*CTX_W+:CTX_W] == wr_ctx);
    end
  endgenerate

  wire [NUM_PINS-1:0] wr_hit = {NUM_PINS{wr_en}} & wr_mask & own;
  wire [NUM_PINS-1:0] oe_hit = {NUM_PINS{oe_en}} & oe_mask & own;
  wire denied = (wr_en & |(wr_mask & ~own)) | (oe_en & |(oe_mask & ~own));

  always @(posedge clk) begin
    if (rst) begin
      out_r     <= OUT_RESET[NUM_PINS-1:0];
      oe_r      <= {NUM_PINS{1'b0}};
      owner_err <= 1'b0;
    end else begin
      out_r <= (out_r & ~wr_hit) | (wr_val & wr_hit);
      oe_r  <= (oe_r & ~oe_hit) | (oe_val & oe_hit);
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
  // * Pad mapping: open-drain pins never drive high (pad_out = 0, oe = !out);
  //   push-pull pins drive out/oe directly.
  // * Ownership: a pin's out/oe register only changes on a write that hits
  //   it (wr_en & mask & owned by the writing context), and then takes the
  //   written value. So a context can never move a pin it does not own.
  // * owner_err: set by any denied bit, cleared by clr_err, otherwise stable.
  // * Reset: out = OUT_RESET, oe = 0, no error.
  // * Idle lemma: until the first write request (wr_en or oe_en) after reset,
  //   out = OUT_RESET and oe = 0, so no push-pull pin drives and open-drain
  //   pins are released.
`ifdef PE_PINS_TOP
  initial assume (rst);
`endif
  reg f_rst_seen = 1'b0;
  always @(posedge clk) if (rst) f_rst_seen <= 1'b1;
  wire f_ok = f_rst_seen & ~rst;
  reg  f_past_ok = 1'b0;
  always @(posedge clk) f_past_ok <= f_ok;

  assign f_out_r = out_r;
  assign f_oe_r  = oe_r;

  reg f_written;
  always @(posedge clk) begin
    if (rst) f_written <= 1'b0;
    else if (wr_en || oe_en) f_written <= 1'b1;
  end

  always @(*) begin
    if (f_ok && !f_written) begin
      assert (out_r == OUT_RESET[NUM_PINS-1:0]);
      assert (oe_r == {NUM_PINS{1'b0}});
    end
  end

  always @(*) begin
    if (f_ok) begin
      assert ((pad_out & pin_od) == {NUM_PINS{1'b0}});
      assert ((pad_oe & pin_od) == (~out_r & pin_od));
      assert ((pad_oe & ~pin_od) == (oe_r & ~pin_od));
      assert ((pad_out & ~pin_od) == (out_r & ~pin_od));
    end
  end

  always @(posedge clk) begin
    if (f_past_ok && f_ok) begin
      assert (((out_r ^ $past(out_r)) & ~$past(wr_hit)) == {NUM_PINS{1'b0}});
      assert ((out_r & $past(wr_hit)) == ($past(wr_val) & $past(wr_hit)));
      assert (((oe_r ^ $past(oe_r)) & ~$past(oe_hit)) == {NUM_PINS{1'b0}});
      assert ((oe_r & $past(oe_hit)) == ($past(oe_val) & $past(oe_hit)));
      assert (($past(wr_hit | oe_hit) & ~$past(own)) == {NUM_PINS{1'b0}});
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
