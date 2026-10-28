/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_host_if: 4-bit host bus -> single-cycle register accesses.
 *
 * Handshake: the host sets h_op and h_din, then toggles h_stb. h_stb is
 * synchronised and edge-detected. h_op and h_din are sampled at that edge,
 * so they must be stable from before the toggle until h_ack toggles.
 * h_ack toggles once each op is complete; h_dout is valid when it does.
 * The host may be slow and asynchronous (RP2040 PIO or GPIO).
 *
 * h_op:
 *   0 ADDR : addr <= {addr[3:0], din}        (two ops: high nibble, then low)
 *   1 WRITE: first nibble = high half; second nibble -> r_we with {hi, din}
 *   2 READ : first read -> r_re, latch r_rdata, dout = high half;
 *            second read -> dout = low half
 *   3 SYNC : reset the write/read nibble phase (resynchronise)
 *
 * The address does not auto-increment. FIFO and IMEM data registers are
 * written repeatedly at the same address.
 */

`default_nettype none

module pe_host_if (
    input wire clk,
    input wire rst,

    // host pins
    input  wire [3:0] h_din,
    input  wire [1:0] h_op,
    input  wire       h_stb,
    output reg  [3:0] h_dout,
    output reg        h_ack,
    output wire       h_irq,

    // register bus
    output reg  [7:0] r_addr,
    output wire [7:0] r_wdata,
    output wire       r_we,
    output wire       r_re,
    input  wire [7:0] r_rdata,
    input  wire       irq
);

  localparam [1:0] OP_ADDR = 2'd0;
  localparam [1:0] OP_WRITE = 2'd1;
  localparam [1:0] OP_READ = 2'd2;
  localparam [1:0] OP_SYNC = 2'd3;

  wire stb_s;
  reg  stb_q;

  pe_sync #(
      .W(1)
  ) u_sync_stb (
      .clk(clk),
      .rst(rst),
      .d  (h_stb),
      .q  (stb_s)
  );

  wire ev = stb_s ^ stb_q;

  reg [3:0] wbuf;
  reg       wphase;
  reg [3:0] rlow;
  reg       rphase;

  assign r_wdata = {wbuf, h_din};
  assign r_we    = ev & (h_op == OP_WRITE) & wphase;
  assign r_re    = ev & (h_op == OP_READ) & ~rphase;
  assign h_irq   = irq;

  always @(posedge clk) begin
    if (rst) begin
      stb_q  <= 1'b0;
      h_ack  <= 1'b0;
      h_dout <= 4'd0;
      r_addr <= 8'd0;
      wbuf   <= 4'd0;
      wphase <= 1'b0;
      rlow   <= 4'd0;
      rphase <= 1'b0;
    end else begin
      stb_q <= stb_s;
      if (ev) begin
        h_ack <= ~h_ack;
        case (h_op)
          OP_ADDR: r_addr <= {r_addr[3:0], h_din};
          OP_WRITE: begin
            if (!wphase) wbuf <= h_din;
            wphase <= ~wphase;
          end
          OP_READ: begin
            if (!rphase) begin
              h_dout <= r_rdata[7:4];
              rlow   <= r_rdata[3:0];
            end else begin
              h_dout <= rlow;
            end
            rphase <= ~rphase;
          end
          OP_SYNC: begin
            wphase <= 1'b0;
            rphase <= 1'b0;
          end
        endcase
      end
    end
  end

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // Host protocol (ALWAYS assumed: these inputs are chip pins): the host
  // holds h_stb low during reset; after toggling h_stb it waits for h_ack
  // to toggle, keeping h_stb, h_op and h_din stable meanwhile.
  // * h_ack == stb_q: the chip acks every strobe edge exactly once, and
  //   never acks without an outstanding request.
  // * Bounded response: once h_stb has been stable for 3 cycles, h_ack == h_stb.
  // * r_we only on the 2nd write nibble, r_re only on the 1st read nibble,
  //   never both; r_addr changes only on an ADDR op, h_dout only on READ.
`ifdef PE_HOST_IF_TOP
  initial assume (rst);
`endif
  reg f_rst_seen = 1'b0;
  always @(posedge clk) if (rst) f_rst_seen <= 1'b1;
  wire f_ok = f_rst_seen & ~rst;
  reg  f_past_ok = 1'b0;
  always @(posedge clk) f_past_ok <= f_ok;

  reg f_pv = 1'b0;
  always @(posedge clk) f_pv <= 1'b1;
  always @(*) if (rst) assume (!h_stb);
  always @(posedge clk) begin
    if (f_pv && f_rst_seen && $past(h_stb != h_ack) && !$past(rst))
      assume (h_stb == $past(h_stb) && h_op == $past(h_op) && h_din == $past(h_din));
  end

  reg [1:0] f_stable;
  always @(posedge clk) begin
    if (rst || (f_pv && h_stb != $past(h_stb))) f_stable <= 2'd0;
    else if (f_stable != 2'd3) f_stable <= f_stable + 1'b1;
  end

  always @(*) begin
    if (f_ok) begin
      assert (h_ack == stb_q);
      assert (!(r_we && r_re));
      if (r_we) assert (ev && h_op == OP_WRITE && wphase);
      if (r_re) assert (ev && h_op == OP_READ && !rphase);
`ifdef PE_HOST_IF_TOP  // module-level covers only when this module is the proof top
      cover (r_we);
      cover (r_re);
`endif
    end
  end

  always @(posedge clk) begin
    if (f_past_ok && f_ok) begin
      if (f_stable == 2'd3 && h_stb == $past(h_stb)) assert (h_ack == h_stb);
      if (h_ack != $past(h_ack)) assert ($past(h_stb != h_ack));
      if (r_addr != $past(r_addr)) assert ($past(ev && h_op == OP_ADDR));
      if (h_dout != $past(h_dout)) assert ($past(ev && h_op == OP_READ));
    end
  end
`endif

endmodule
