/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_icg: integrated clock gate for the memories (MEM_CLOCK_GATE in the top).
 *
 * gclk = clk & en_l, where en_l is `en` captured by a latch that is open while
 * clk is low. So gclk only has a rising edge in a cycle whose `en` was 1 just
 * before the edge, and it can never glitch: `en` may change at any time while
 * clk is high. A flip-flop on gclk therefore behaves exactly like a flip-flop
 * on clk with enable `en`, without the 2:1 hold multiplexer.
 *
 * Synthesis (Yosys defines SYNTHESIS) uses the IHP cell sg13g2_lgcp_1, which
 * is this circuit. Simulation and lint use the same logic written out.
 */

`default_nettype none

module pe_icg (
    input  wire clk,
    input  wire en,
    output wire gclk
);

`ifdef SYNTHESIS
  sg13g2_lgcp_1 u_cg (
      .CLK (clk),
      .GATE(en),
      .GCLK(gclk)
  );
`else
  reg en_l;
  /* verilator lint_off LATCH */
  always @(*) begin
    if (!clk) en_l = en;
  end
  /* verilator lint_on LATCH */
  assign gclk = clk & en_l;
`endif

endmodule
