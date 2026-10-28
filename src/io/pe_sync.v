/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_sync: W-bit 2-FF synchroniser for asynchronous inputs (pins, host strobe).
 */

`default_nettype none

module pe_sync #(
    parameter W         = 1,
    parameter RESET_VAL = 0
) (
    input  wire         clk,
    input  wire         rst,
    input  wire [W-1:0] d,
    output wire [W-1:0] q
);

  reg [W-1:0] s1, s2;

  always @(posedge clk) begin
    if (rst) begin
      s1 <= RESET_VAL[W-1:0];
      s2 <= RESET_VAL[W-1:0];
    end else begin
      s1 <= d;
      s2 <= s1;
    end
  end

  assign q = s2;

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // q is d delayed by exactly 2 cycles; a reset forces RESET_VAL.
  reg f_v1 = 1'b0, f_v2 = 1'b0;
  always @(posedge clk) begin
    f_v1 <= 1'b1;
    f_v2 <= f_v1;
  end

  always @(posedge clk) begin
    if (f_v1 && $past(rst)) assert (q == RESET_VAL[W-1:0]);
    if (f_v2 && !$past(rst) && !$past(rst, 2)) assert (q == $past(d, 2));
  end
`endif

endmodule
