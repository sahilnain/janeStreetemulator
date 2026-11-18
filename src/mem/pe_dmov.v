/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_dmov: the DMEM port and the two data movers (docs/spec/blocks.md Section 14).
 *
 * DMEM (pe_dmem) has one port with a registered read. Four users share it,
 * one access per cycle, in this priority:
 *   1. host    DMEM_DATA access from pe_regs (h_acc; h_we = write, else read)
 *   2. RXM     the RX mover's write
 *   3. TXM     the TX mover's read
 *   4. controller `ld` (c_re) / `st` (c_we); c_gnt = 1 when it gets the port
 *      (else it retries next cycle, like the other blocked instructions)
 *
 * TX mover (unit 5): copies DMEM[ptr..] to the TX FIFO, one byte at a time,
 * on its own. Started by `ucfg TXM, MV_GO, n` (n = 1..256 bytes; 0 stops it).
 * Per byte: read DMEM (when the port is free), take the byte into a 1-byte
 * buffer the next cycle, then push it into the TX FIFO when it is not full
 * and the host is not writing the FIFO in that cycle. Busy until the last
 * byte has been pushed.
 *
 * RX mover (unit 6): copies RX FIFO bytes to DMEM[ptr..]. Started by
 * `ucfg RXM, MV_GO, n` (n = the most bytes to take, 1..256; 0 stops it). Each
 * cycle the RX FIFO is not empty, the host is not popping it and the port is
 * free, it pops one byte and writes it. `count` = bytes written since GO.
 *
 * Config (ucfg unit 5/6): idx 0 PTR = data[7:0]; idx 1 GO = data[8:0].
 * Read (urd unit 5): idx 0 ptr, 1 bytes left. (unit 6): 0 ptr, 1 count, 2 left.
 * restart and reset stop both movers.
 */

`default_nettype none

module pe_dmov (
    input wire clk,
    input wire rst,
    input wire restart,

    // host (pe_regs)
    input wire       h_acc,
    input wire       h_we,
    input wire [7:0] h_addr,
    input wire [7:0] h_wdata,

    // controller ld / st
    input  wire       c_re,
    input  wire       c_we,
    input  wire [7:0] c_addr,
    input  wire [7:0] c_wdata,
    output wire       c_gnt,

    // controller config and read-back
    input  wire        cfg_we,
    input  wire [ 2:0] cfg_unit,
    input  wire [ 2:0] cfg_idx,
    input  wire [15:0] cfg_data,
    input  wire [ 2:0] rd_unit,
    input  wire [ 2:0] rd_idx,
    output reg  [15:0] rd_data,

    // TX FIFO write side (the host writes first)
    input  wire       h_txf_wr,
    input  wire       txf_full,
    output wire       m_txf_wr,
    output wire [7:0] m_txf_wdata,

    // RX FIFO read side (the host reads first)
    input  wire       h_rxf_rd,
    input  wire       rxf_empty,
    input  wire [7:0] rxf_rdata,
    output wire       m_rxf_rd,

    output wire txm_busy,
    output wire rxm_busy,

    // DMEM port
    output wire       m_re,
    output wire       m_we,
    output wire [7:0] m_addr,
    output wire [7:0] m_wdata,
    input  wire [7:0] m_rdata
);

  localparam [2:0] U_TXM = 3'd5, U_RXM = 3'd6;
  localparam [2:0] MV_PTR = 3'd0, MV_GO = 3'd1;

  // ---------------------------------------------------------------- state
  reg [7:0] t_ptr, r_ptr;
  reg [8:0] t_left, r_left, r_cnt;
  reg       t_pend;  // a TXM read went out last cycle: the byte is on m_rdata now
  reg       t_bufv;
  reg [7:0] t_buf;

  // ---------------------------------------------------------- port arbitration
  wire r_req = (r_left != 9'd0) & ~rxf_empty & ~h_rxf_rd;
  wire r_go = r_req & ~h_acc;  // RXM pops and writes this cycle
  wire t_req = (t_left != 9'd0) & ~t_pend & ~t_bufv;
  wire t_go = t_req & ~h_acc & ~r_req;  // TXM reads this cycle
  assign c_gnt = ~h_acc & ~r_req & ~t_req;

  assign m_re = h_acc ? ~h_we : t_go | (c_gnt & c_re);
  assign m_we = h_acc ? h_we : r_go | (c_gnt & c_we);
  assign m_addr = h_acc ? h_addr : r_req ? r_ptr : t_req ? t_ptr : c_addr;
  assign m_wdata = h_acc ? h_wdata : r_req ? rxf_rdata : c_wdata;

  assign m_rxf_rd = r_go;
  assign m_txf_wr = t_bufv & ~txf_full & ~h_txf_wr;
  assign m_txf_wdata = t_buf;

  assign txm_busy = (t_left != 9'd0) | t_pend | t_bufv;
  assign rxm_busy = (r_left != 9'd0);

  wire cfg_t = cfg_we & (cfg_unit == U_TXM);
  wire cfg_r = cfg_we & (cfg_unit == U_RXM);

  // ------------------------------------------------------------- registers
  always @(posedge clk) begin
    if (rst || restart) begin
      t_ptr  <= 8'd0;
      t_left <= 9'd0;
      t_pend <= 1'b0;
      t_bufv <= 1'b0;
      t_buf  <= 8'd0;
      r_ptr  <= 8'd0;
      r_left <= 9'd0;
      r_cnt  <= 9'd0;
    end else begin
      // TX mover
      if (cfg_t && cfg_idx == MV_PTR) t_ptr <= cfg_data[7:0];
      else if (t_go) t_ptr <= t_ptr + 1'b1;
      if (cfg_t && cfg_idx == MV_GO) begin
        t_left <= cfg_data[8:0];
        t_pend <= 1'b0;
        t_bufv <= 1'b0;
      end else begin
        if (t_go) t_left <= t_left - 1'b1;
        t_pend <= t_go;
        if (t_pend) begin
          t_buf  <= m_rdata;
          t_bufv <= 1'b1;
        end else if (m_txf_wr) t_bufv <= 1'b0;
      end
      // RX mover
      if (cfg_r && cfg_idx == MV_PTR) r_ptr <= cfg_data[7:0];
      else if (r_go) r_ptr <= r_ptr + 1'b1;
      if (cfg_r && cfg_idx == MV_GO) begin
        r_left <= cfg_data[8:0];
        r_cnt  <= 9'd0;
      end else if (r_go) begin
        r_left <= r_left - 1'b1;
        r_cnt  <= r_cnt + 1'b1;
      end
    end
  end

  always @(*) begin
    rd_data = 16'd0;
    if (rd_unit == U_TXM) begin
      case (rd_idx)
        3'd0: rd_data = {8'd0, t_ptr};
        3'd1: rd_data = {7'd0, t_left};
        default: ;
      endcase
    end else if (rd_unit == U_RXM) begin
      case (rd_idx)
        3'd0: rd_data = {8'd0, r_ptr};
        3'd1: rd_data = {7'd0, r_cnt};
        3'd2: rd_data = {7'd0, r_left};
        default: ;
      endcase
    end
  end

  wire _unused = &{1'b0, cfg_data[15:9]};

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // * One access per DMEM cycle, never a read and a write together; the host
  //   always gets the port; the controller only when nobody else uses it.
  // * The RX mover pops only a non-empty FIFO the host is not popping, and
  //   writes exactly the popped byte; the TX mover pushes only into a FIFO
  //   that is not full and that the host is not writing.
  // * TXM: a read is followed by the byte in the buffer the next cycle; at most
  //   one byte is in flight; busy until the last byte is pushed.
  reg f_rst_seen = 1'b0;
  always @(posedge clk) if (rst) f_rst_seen <= 1'b1;
  wire f_ok = f_rst_seen & ~rst;
  reg  f_past_ok = 1'b0;
  always @(posedge clk) f_past_ok <= f_ok;
`ifdef PE_DMOV_TOP
  initial assume (rst);
  // the controller never sends ld and st at once (formal/pe_ctrl.sby: dm_re only on ld, dm_we only on st)
  always @(*) assume (!(c_re && c_we));
`endif
  always @(*) begin
    if (f_ok) begin
      assert (!(m_re && m_we));
      if (h_acc) assert (m_re == !h_we && m_we == h_we && m_addr == h_addr);
      if (c_gnt) assert (!h_acc && !r_go && !t_go);
      if (m_rxf_rd) assert (!rxf_empty && !h_rxf_rd && m_we && m_wdata == rxf_rdata && !h_acc);
      if (m_txf_wr) assert (!txf_full && !h_txf_wr && t_bufv);
      assert (!(t_pend && t_bufv));
      if (t_go) assert (m_re && m_addr == t_ptr);
      assert (txm_busy == (t_left != 0 || t_pend || t_bufv));
    end
  end
  always @(posedge clk) begin
    if (f_past_ok && f_ok && !$past(rst) && !$past(restart) && !$past(cfg_t)) begin
      if ($past(t_pend)) assert (t_bufv && t_buf == $past(m_rdata));
      if ($past(t_go)) assert (t_pend && t_ptr == $past(t_ptr) + 8'd1);
    end
    if (f_past_ok && f_ok && !$past(rst) && !$past(restart) && !$past(cfg_r)) begin
      if ($past(r_go)) assert (r_cnt == $past(r_cnt) + 9'd1 && r_left == $past(r_left) - 9'd1);
      else assert (r_cnt == $past(r_cnt) && r_left == $past(r_left));
    end
  end
`ifdef PE_DMOV_TOP
  always @(*) begin
    cover (m_txf_wr);
    cover (m_rxf_rd);
    cover (c_gnt && c_re);
  end
`endif
`endif

endmodule
