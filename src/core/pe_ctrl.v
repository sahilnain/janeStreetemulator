/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_ctrl: the controller (docs/spec/blocks.md Section 2).
 *
 * One instruction stream. The front end is v1's: pe_imem has a registered
 * read, so the controller fetches in one cycle and executes in the next. A
 * slot bit alternates every cycle and the controller may only issue in
 * slot 0, so it issues at most every 2 cycles (cycle E = execute cycle).
 *
 * State: pc, r0-r7 (16 bits), flags Z C TO DB, return stack, stall counter.
 * Every effect of an instruction (registers, pins, FIFOs, unit commands) is
 * written in E. An instruction that cannot complete is "blocked": it has no
 * effect at all and is retried at E+2 (wait, fget/fput, ld/st, ustart,
 * uwait, evt wait). A blocked `wait` with a timeout still counts rd down.
 *
 * Stall: `dly` d (plus op2 for `delay`) loads the stall counter, which
 * counts down every cycle; the next issue is at E + 2*ceil((d+1)/2).
 *
 * restart (host RESTART): pc <- start_pc, registers/flags/stack cleared,
 * stall = 1 so the word already fetched from the old pc is not issued.
 * DB is a host doorbell: set by db_set, cleared by `host dbclr`.
 */

`default_nettype none

module pe_ctrl #(
    parameter IMEM_DEPTH  = 64,
    parameter NUM_PINS    = 8,
    parameter STACK_DEPTH = 4,
    // derived, do not override
    parameter AW          = (IMEM_DEPTH > 1) ? $clog2(IMEM_DEPTH) : 1
) (
    input wire clk,
    input wire rst,
`ifdef PE_CHIP_FORMAL
    input wire f_seen_i,
`endif

    // control from pe_regs
    input wire          run,
    input wire          restart,
    input wire [AW-1:0] start_pc,
    input wire          db_set,

    // instruction memory
    output wire [AW-1:0] im_raddr,
    input  wire [  31:0] im_rdata,

    // pins: CTRL write port of pe_pins, synchronised inputs
    output reg  [NUM_PINS-1:0] pw_mask,
    output reg  [NUM_PINS-1:0] pw_val,
    output reg  [NUM_PINS-1:0] po_mask,
    output reg  [NUM_PINS-1:0] po_val,
    input  wire [NUM_PINS-1:0] pin_in,

    // FIFOs (a request is only made when it is granted)
    input  wire       txf_empty,
    input  wire       txf_full,
    input  wire [7:0] txf_rdata,
    input  wire       rxf_empty,
    input  wire       rxf_full,
    input  wire [7:0] rxf_rdata,
    input  wire       txf_rd_gnt,
    input  wire       txf_wr_gnt,
    input  wire       rxf_rd_gnt,
    input  wire       rxf_wr_gnt,
    output reg        txf_rd,
    output reg        rxf_rd,
    output reg        txf_wr,
    output reg        rxf_wr,
    output wire [7:0] f_wdata,

    // data memory
    output wire [7:0] dm_addr,
    input  wire [7:0] dm_rdata,
    output reg        dm_we,
    output wire [7:0] dm_wdata,
    input  wire       dm_gnt,

    // units and detectors
    output reg         cfg_we,
    output wire [ 2:0] cfg_unit,
    output wire [ 2:0] cfg_idx,
    output wire [15:0] cfg_data,
    output wire [ 2:0] rd_unit,
    output wire [ 2:0] rd_idx,
    input  wire [15:0] rd_data,
    output reg         rd_strobe,
    output reg  [ 1:0] start_mask,
    output reg         start_imm,
    output wire [15:0] start_data,
    output reg  [ 1:0] stop_mask,
    input  wire        tx_busy,
    input  wire        rx_busy,
    input  wire        tx_need,     // TX unit needs a byte at start (DEN)
    input  wire        rx_need,     // RX unit needs a byte at start (drive mode)
    output wire        det_sel,     // 0 ETX, 1 ERX
    output reg         det_clr,
    output reg         det_arm,
    output reg         det_disarm,
    input  wire        etx_flag,
    input  wire        erx_flag,
    input  wire        ferr,
    input  wire        rxrdy,

    // host signalling and errors (1-cycle strobes)
    output reg host_irq,
    output reg ill_err,
    output reg stk_err,

    // status
    output reg  [AW-1:0] pc,
    output wire          z_o,
    output wire          c_o,
    output wire          to_o,
    output wire          db_o,
    output reg           blocked
);

  localparam STALL_W = 17;
  localparam SP_W = $clog2(STACK_DEPTH + 1);
  localparam SI_W = (STACK_DEPTH > 1) ? $clog2(STACK_DEPTH) : 1;
  localparam integer STACK_DEPTH_I = STACK_DEPTH;
  localparam [SP_W-1:0] SP_FULL = STACK_DEPTH_I[SP_W-1:0];
  localparam WX = NUM_PINS + 16;

  // opcodes
  localparam [5:0] OP_NOP = 6'd0, OP_JMP = 6'd1, OP_DJNZ = 6'd2, OP_CALL = 6'd3, OP_RET = 6'd4;
  localparam [5:0] OP_DELAY = 6'd6, OP_MOV = 6'd7, OP_ADD = 6'd8, OP_SUB = 6'd9, OP_AND = 6'd10;
  localparam [5:0] OP_OR = 6'd11, OP_XOR = 6'd12, OP_SHL = 6'd13, OP_SHR = 6'd14, OP_CMP = 6'd15;
  localparam [5:0] OP_SET = 6'd16, OP_SETOE = 6'd17, OP_WAIT = 6'd18, OP_JPIN = 6'd19;
  localparam [5:0] OP_RDPINS = 6'd20, OP_BITIN = 6'd21, OP_BITOUT = 6'd22;
  localparam [5:0] OP_FGET = 6'd23, OP_FPUT = 6'd24, OP_LD = 6'd25, OP_ST = 6'd26;
  localparam [5:0] OP_UCFG = 6'd27, OP_URD = 6'd28, OP_USTART = 6'd29, OP_USTOP = 6'd30;
  localparam [5:0] OP_UWAIT = 6'd31, OP_EVT = 6'd32, OP_HOST = 6'd33, OP_PAR = 6'd34;

  // ---------------------------------------------------------------- state
  reg                slot;
  reg  [       15:0] r        [0:7];
  reg                z, c, to, db;
  reg  [   AW-1:0]   stack    [0:STACK_DEPTH-1];
  reg  [   SP_W-1:0] sp;
  reg  [STALL_W-1:0] stall;

  wire               ready = run & ~slot & (stall == {STALL_W{1'b0}});
  assign im_raddr = pc;

  // --------------------------------------------------------------- decode
  wire [5:0] opcode;
  wire [2:0] dly, fa, fb, rs_i;
  wire       fi, lvl, illegal;
  wire [15:0] imm;
  wire [4:0] cond;
  wire [AW-1:0] target;
  wire [3:0] pin;

  pe_decode #(
      .AW(AW)
  ) u_decode (
      .ir     (im_rdata),
      .opcode (opcode),
      .dly    (dly),
      .fa     (fa),
      .fi     (fi),
      .fb     (fb),
      .imm    (imm),
      .cond   (cond),
      .target (target),
      .pin    (pin),
      .lvl    (lvl),
      .rs     (rs_i),
      .illegal(illegal)
  );

  wire [15:0] ra_v = r[fa];  // A operand (rd / rs)
  wire [15:0] rb_v = r[fb];  // B operand (ra for ld/st)
  wire [15:0] op2 = fi ? imm : r[rs_i];

  // ---------------------------------------------------------------- pins
  wire [WX-1:0] pin_x = {16'd0, pin_in} >> pin;
  wire pin_bit = pin_x[0];
  wire [WX-1:0] one_x = {{(WX - 1) {1'b0}}, 1'b1} << pin;
  wire [NUM_PINS-1:0] one_mask = one_x[NUM_PINS-1:0];
  wire [15:0] pins16;
  generate
    if (NUM_PINS >= 16) begin : g_p16
      assign pins16 = pin_in[15:0];
    end else begin : g_plt
      assign pins16 = {{(16 - NUM_PINS) {1'b0}}, pin_in};
    end
  endgenerate

  // ---------------------------------------------------------- conditions
  reg cond_ok;
  always @(*) begin
    case (cond)
      5'd0: cond_ok = 1'b1;
      5'd1: cond_ok = z;
      5'd2: cond_ok = ~z;
      5'd3: cond_ok = c;
      5'd4: cond_ok = ~c;
      5'd5: cond_ok = to;
      5'd6: cond_ok = ~to;
      5'd7: cond_ok = erx_flag;
      5'd8: cond_ok = ~erx_flag;
      5'd9: cond_ok = tx_busy;
      5'd10: cond_ok = ~tx_busy;
      5'd11: cond_ok = rx_busy;
      5'd12: cond_ok = ~rx_busy;
      5'd13: cond_ok = etx_flag;
      5'd14: cond_ok = ~etx_flag;
      5'd15: cond_ok = ferr;
      5'd16: cond_ok = ~ferr;
      5'd17: cond_ok = txf_empty;
      5'd18: cond_ok = ~txf_empty;
      5'd19: cond_ok = txf_full;
      5'd20: cond_ok = rxf_empty;
      5'd21: cond_ok = ~rxf_empty;
      5'd22: cond_ok = rxf_full;
      5'd23: cond_ok = db;
      5'd24: cond_ok = ~db;
      5'd25: cond_ok = rxrdy;
      5'd26: cond_ok = ~rxrdy;
      default: cond_ok = 1'b0;
    endcase
  end

  // ----------------------------------------------------------------- ALU
  wire [16:0] add_r = {1'b0, ra_v} + {1'b0, op2};
  wire [16:0] sub_r = {1'b0, ra_v} - {1'b0, op2};
  wire [3:0] sh_n = imm[3:0];
  wire [31:0] shl_x = {16'd0, ra_v} << sh_n;  // bit 16 = last bit out
  wire [31:0] shr_x = {ra_v, 16'd0} >> sh_n;  // bit 15 = last bit out
  wire [15:0] dec_a = ra_v - 1'b1;

  // ---------------------------------------------------- unit / FIFO glue
  assign cfg_unit   = fa;
  assign cfg_idx    = fb;
  assign cfg_data   = op2;
  assign rd_unit    = fb;
  assign rd_idx     = imm[2:0];
  assign start_data = op2;
  assign det_sel    = fa[0];
  assign f_wdata    = ra_v[7:0];
  assign dm_addr    = rb_v[7:0];
  assign dm_wdata   = ra_v[7:0];

  wire       f_rx = fb[0];  // fifo select: 0 TX, 1 RX
  wire       nb = imm[0];
  wire [1:0] umask = fa[1:0];
  wire       ubusy = (umask[0] & tx_busy) | (umask[1] & rx_busy);
  wire       need_byte = (umask[0] & tx_need) | (umask[1] & rx_need);
  wire       evt_flag = fa[0] ? erx_flag : etx_flag;

  // ---------------------------------------------------- next-state logic
  reg              done, jump;
  reg [STALL_W-1:0] stall_n;
  reg              we_a, we_b;  // register writes: a = rd, b = ra (post-inc)
  reg [       2:0] wa_i, wb_i;
  reg [      15:0] wa_v, wb_v;
  reg              z_n, c_n, to_n, db_clr;
  reg              push, pop;
  reg              ill_n, stk_n;

  always @(*) begin
    done       = 1'b1;
    jump       = 1'b0;
    we_a       = 1'b0;
    wa_i       = fa;
    wa_v       = 16'd0;
    we_b       = 1'b0;
    wb_i       = fb;
    wb_v       = rb_v + 1'b1;
    z_n        = z;
    c_n        = c;
    to_n       = to;
    db_clr     = 1'b0;
    push       = 1'b0;
    pop        = 1'b0;
    ill_n      = 1'b0;
    stk_n      = 1'b0;
    pw_mask    = {NUM_PINS{1'b0}};
    pw_val     = {NUM_PINS{1'b0}};
    po_mask    = {NUM_PINS{1'b0}};
    po_val     = {NUM_PINS{1'b0}};
    txf_rd     = 1'b0;
    rxf_rd     = 1'b0;
    txf_wr     = 1'b0;
    rxf_wr     = 1'b0;
    dm_we      = 1'b0;
    cfg_we     = 1'b0;
    rd_strobe  = 1'b0;
    start_mask = 2'b00;
    start_imm  = fb[0];
    stop_mask  = 2'b00;
    det_clr    = 1'b0;
    det_arm    = 1'b0;
    det_disarm = 1'b0;
    host_irq   = 1'b0;

    case (opcode)
      OP_NOP, OP_DELAY: ;
      OP_JMP: jump = cond_ok;
      OP_DJNZ: begin
        we_a = 1'b1;
        wa_v = dec_a;
        jump = (dec_a != 16'd0);
      end
      OP_CALL: begin
        if (sp == SP_FULL) stk_n = 1'b1;
        else begin
          push = 1'b1;
          jump = 1'b1;
        end
      end
      OP_RET: begin
        if (sp == {SP_W{1'b0}}) stk_n = 1'b1;
        else pop = 1'b1;
      end
      OP_MOV: begin
        we_a = 1'b1;
        wa_v = op2;
      end
      OP_ADD: begin
        we_a = 1'b1;
        wa_v = add_r[15:0];
        z_n  = (add_r[15:0] == 16'd0);
        c_n  = add_r[16];
      end
      OP_SUB, OP_CMP: begin
        we_a = (opcode == OP_SUB);
        wa_v = sub_r[15:0];
        z_n  = (sub_r[15:0] == 16'd0);
        c_n  = sub_r[16];
      end
      OP_AND, OP_OR, OP_XOR: begin
        we_a = 1'b1;
        wa_v = (opcode == OP_AND) ? (ra_v & op2) : (opcode == OP_OR) ? (ra_v | op2) : (ra_v ^ op2);
        z_n  = (wa_v == 16'd0);
      end
      OP_SHL: begin
        we_a = 1'b1;
        wa_v = shl_x[15:0];
        z_n  = (shl_x[15:0] == 16'd0);
        c_n  = (sh_n != 4'd0) & shl_x[16];
      end
      OP_SHR: begin
        we_a = 1'b1;
        wa_v = shr_x[31:16];
        z_n  = (shr_x[31:16] == 16'd0);
        c_n  = (sh_n != 4'd0) & shr_x[15];
      end
      OP_SET: begin
        pw_mask = one_mask;
        pw_val  = {NUM_PINS{imm[1] ? (c ^ imm[0]) : imm[0]}};
      end
      OP_SETOE: begin
        po_mask = one_mask;
        po_val  = {NUM_PINS{imm[0]}};
      end
      OP_WAIT: begin
        if (pin_bit == lvl) begin
          if (fi) to_n = 1'b0;
        end else if (fi && ra_v == 16'd0) begin
          to_n = 1'b1;
        end else begin
          done = 1'b0;
          if (fi) begin
            we_a = 1'b1;
            wa_v = dec_a;
          end
        end
      end
      OP_JPIN: jump = (pin_bit == lvl);
      OP_RDPINS: begin
        we_a = 1'b1;
        wa_v = pins16;
      end
      OP_BITIN: begin
        we_a = 1'b1;
        wa_v = imm[0] ? {ra_v[14:0], pin_bit} : {pin_bit, ra_v[15:1]};
      end
      OP_BITOUT: begin
        pw_mask = one_mask;
        pw_val  = {NUM_PINS{imm[0] ? ra_v[15] : ra_v[0]}};
        we_a    = 1'b1;
        wa_v    = imm[0] ? {ra_v[14:0], 1'b0} : {1'b0, ra_v[15:1]};
      end
      OP_FGET: begin
        if (f_rx ? !rxf_rd_gnt : !txf_rd_gnt) done = 1'b0;
        else if (f_rx ? rxf_empty : txf_empty) begin
          if (nb) c_n = 1'b1;
          else done = 1'b0;
        end else begin
          txf_rd = ~f_rx;
          rxf_rd = f_rx;
          we_a   = 1'b1;
          wa_v   = {8'd0, f_rx ? rxf_rdata : txf_rdata};
          if (nb) c_n = 1'b0;
        end
      end
      OP_FPUT: begin
        if (f_rx ? !rxf_wr_gnt : !txf_wr_gnt) done = 1'b0;
        else if (f_rx ? rxf_full : txf_full) begin
          if (nb) c_n = 1'b1;
          else done = 1'b0;
        end else begin
          txf_wr = ~f_rx;
          rxf_wr = f_rx;
          if (nb) c_n = 1'b0;
        end
      end
      OP_LD, OP_ST: begin
        if (!dm_gnt) done = 1'b0;
        else begin
          dm_we = (opcode == OP_ST);
          we_a  = (opcode == OP_LD);
          wa_v  = {8'd0, dm_rdata};
          we_b  = imm[0];
        end
      end
      OP_UCFG: cfg_we = 1'b1;
      OP_URD: begin
        rd_strobe = 1'b1;
        we_a      = 1'b1;
        wa_v      = rd_data;
      end
      OP_USTART: begin
        if (ubusy || (need_byte && !fb[0] && txf_empty)) done = 1'b0;
        else start_mask = umask;
      end
      OP_USTOP: stop_mask = umask;
      OP_UWAIT: done = ~ubusy;
      OP_EVT: begin
        case (fb)
          3'd0: done = evt_flag;
          3'd1: begin
            done    = evt_flag;
            det_clr = evt_flag;
          end
          3'd2: det_clr = 1'b1;
          3'd3: det_arm = 1'b1;
          3'd4: det_disarm = 1'b1;
          default: ;
        endcase
      end
      OP_HOST: begin
        host_irq = (fb == 3'd0);
        db_clr   = (fb == 3'd1);
      end
      OP_PAR: c_n = ^ra_v;
      default: ill_n = 1'b1;
    endcase

    // A blocked instruction has no side effects (only a timed-out wait's
    // counter, set above, may change).
    if (!done) begin
      z_n        = z;
      c_n        = c;
      to_n       = to;
      push       = 1'b0;
      pop        = 1'b0;
      pw_mask    = {NUM_PINS{1'b0}};
      po_mask    = {NUM_PINS{1'b0}};
      txf_rd     = 1'b0;
      rxf_rd     = 1'b0;
      txf_wr     = 1'b0;
      rxf_wr     = 1'b0;
      dm_we      = 1'b0;
      start_mask = 2'b00;
      det_clr    = 1'b0;
      we_b       = 1'b0;
      if (opcode != OP_WAIT) we_a = 1'b0;
    end

    if (done)
      stall_n = {{(STALL_W - 3) {1'b0}}, dly} +
                ((opcode == OP_DELAY) ? {{(STALL_W - 16) {1'b0}}, op2} : {STALL_W{1'b0}});
    else stall_n = {STALL_W{1'b0}};

    // Gate every effect with the issue condition.
    if (!ready) begin
      pw_mask    = {NUM_PINS{1'b0}};
      po_mask    = {NUM_PINS{1'b0}};
      txf_rd     = 1'b0;
      rxf_rd     = 1'b0;
      txf_wr     = 1'b0;
      rxf_wr     = 1'b0;
      dm_we      = 1'b0;
      cfg_we     = 1'b0;
      rd_strobe  = 1'b0;
      start_mask = 2'b00;
      stop_mask  = 2'b00;
      det_clr    = 1'b0;
      det_arm    = 1'b0;
      det_disarm = 1'b0;
      host_irq   = 1'b0;
    end
    ill_err = ready & ill_n;
    stk_err = ready & stk_n;
  end

  wire [AW-1:0] pc_inc = pc + 1'b1;
  wire [SP_W-1:0] sp_dec = sp - 1'b1;
  wire [AW-1:0] pc_n = !done ? pc : pop ? stack[sp_dec[SI_W-1:0]] : jump ? target : pc_inc;

  // ------------------------------------------------------------ registers
  integer i;
  always @(posedge clk) begin
    if (rst) begin
      slot    <= 1'b0;
      pc      <= {AW{1'b0}};
      z       <= 1'b0;
      c       <= 1'b0;
      to      <= 1'b0;
      db      <= 1'b0;
      sp      <= {SP_W{1'b0}};
      stall   <= {STALL_W{1'b0}};
      blocked <= 1'b0;
      for (i = 0; i < 8; i = i + 1) r[i] <= 16'd0;
    end else begin
      slot <= ~slot;
      if (db_set) db <= 1'b1;
      else if (ready && db_clr) db <= 1'b0;
      if (restart) begin
        pc      <= start_pc;
        z       <= 1'b0;
        c       <= 1'b0;
        to      <= 1'b0;
        sp      <= {SP_W{1'b0}};
        stall   <= {{(STALL_W - 1) {1'b0}}, 1'b1};
        blocked <= 1'b0;
        for (i = 0; i < 8; i = i + 1) r[i] <= 16'd0;
      end else if (ready) begin
        pc      <= pc_n;
        z       <= z_n;
        c       <= c_n;
        to      <= to_n;
        stall   <= stall_n;
        blocked <= ~done;
        if (we_b) r[wb_i] <= wb_v;
        if (we_a) r[wa_i] <= wa_v;  // rd wins over the post-increment of ra
        if (push) begin
          stack[sp[SI_W-1:0]] <= pc_inc;
          sp        <= sp + 1'b1;
        end
        if (pop) sp <= sp - 1'b1;
      end else if (stall != {STALL_W{1'b0}}) begin
        stall <= stall - 1'b1;
      end
    end
  end

  assign z_o  = z;
  assign c_o  = c;
  assign to_o = to;
  assign db_o = db;

  // `illegal` = the default case of the opcode decode above
  wire _unused = &{1'b0, pin_x[WX-1:1], one_x[WX-1:NUM_PINS], shl_x[31:17], shr_x[14:0], illegal,
                   sp_dec};

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // im_rdata, pins, FIFO flags, grants and all environment inputs are FREE,
  // so these hold for ANY program in ANY environment.
  // * No effect without an issue; issues are at least 2 cycles apart.
  // * A blocked instruction has no side effects, keeps pc and flags, loads no
  //   stall; only wait / fget / fput / ld / st / ustart / uwait / evt can block.
  // * FIFO, DMEM requests only when allowed (never pop empty, never lose an
  //   arbitration); ustart never starts a busy unit, or without a needed byte.
  // * Reserved opcodes: ill_err and nothing else. Stack: sp <= STACK_DEPTH;
  //   overflow / underflow give stk_err and no stack change; `ret` returns to
  //   the pc after the matching `call` (one solver-chosen level is tracked).
  // * A solver-chosen register changes only on a write that targets it.
  // * pc: stays while blocked; pc+1 unless jump / pop. Stall counts down by 1.
  // * restart: pc = start_pc, state cleared, stall = 1.
`ifdef PE_CTRL_TOP
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

  wire f_any_fx = (|pw_mask) | (|po_mask) | txf_rd | rxf_rd | txf_wr | rxf_wr | dm_we | cfg_we |
                  rd_strobe | (|start_mask) | (|stop_mask) | det_clr | det_arm | det_disarm | host_irq;
  wire f_can_block = (opcode == OP_WAIT) | (opcode == OP_FGET) | (opcode == OP_FPUT) |
                     (opcode == OP_LD) | (opcode == OP_ST) | (opcode == OP_USTART) |
                     (opcode == OP_UWAIT) | (opcode == OP_EVT);

  always @(*) begin
    if (f_ok) begin
      assert (sp <= SP_FULL);
      if (!ready) assert (!f_any_fx && !ill_err && !stk_err);
      if (ready && !done) begin
        assert (f_can_block);
        assert (pw_mask == {NUM_PINS{1'b0}} && po_mask == {NUM_PINS{1'b0}});
        assert (!txf_rd && !rxf_rd && !txf_wr && !rxf_wr && !dm_we && !push && !pop && !we_b);
        assert (start_mask == 2'b00 && !det_clr && stall_n == {STALL_W{1'b0}});
        assert (pc_n == pc && z_n == z && c_n == c && to_n == to);
        if (opcode != OP_WAIT) assert (!we_a);
      end
      if (txf_rd) assert (!txf_empty && txf_rd_gnt && opcode == OP_FGET);
      if (rxf_rd) assert (!rxf_empty && rxf_rd_gnt && opcode == OP_FGET);
      if (txf_wr) assert (!txf_full && txf_wr_gnt && opcode == OP_FPUT);
      if (rxf_wr) assert (!rxf_full && rxf_wr_gnt && opcode == OP_FPUT);
      if (dm_we) assert (dm_gnt && opcode == OP_ST);
      if (ready && opcode == OP_LD && done) assert (dm_gnt);
      if (start_mask[0]) assert (!tx_busy && (!tx_need || start_imm || !txf_empty));
      if (start_mask[1]) assert (!rx_busy && (!rx_need || start_imm || !txf_empty));
      if (ready && illegal) begin
        assert (ill_err && !f_any_fx && done && !we_a && !we_b && !push && !pop);
        assert (z_n == z && c_n == c && to_n == to && pc_n == pc_inc);
      end
      if (ill_err) assert (illegal);
      if (stk_err) assert (opcode == OP_CALL || opcode == OP_RET);
      if (push) assert (sp != SP_FULL);
      if (pop) assert (sp != {SP_W{1'b0}});
      if (ready && done && !jump && !pop) assert (pc_n == pc_inc);
`ifdef PE_CTRL_TOP  // module-level covers only when this module is the proof top
      cover (ready && opcode == OP_RET && pop);
      cover (blocked && ready && done);
      cover (stk_err);
      cover (ill_err);
`endif
    end
  end

  // Register file: a solver-chosen register only changes on a write to it.
  (* anyconst *) reg [2:0] f_ri;
  reg [15:0] f_rv;
  always @(posedge clk) f_rv <= r[f_ri];

  // Stack: track the return address pushed at one solver-chosen level.
  (* anyconst *) reg [SP_W-1:0] f_lvl;
  reg          f_trk;
  reg [AW-1:0] f_ret;
  always @(posedge clk) begin
    if (rst || restart) f_trk <= 1'b0;
    else if (ready) begin
      if (push && sp == f_lvl) begin
        f_trk <= 1'b1;
        f_ret <= pc_inc;
      end else if (pop && sp_dec == f_lvl) f_trk <= 1'b0;
    end
  end
  always @(*) begin
    if (f_ok && f_trk) begin
      assert (sp > f_lvl);
      assert (stack[f_lvl[SI_W-1:0]] == f_ret);
      if (ready && pop && sp_dec == f_lvl) assert (pc_n == f_ret);
    end
  end

  always @(posedge clk) begin
    if (f_past_ok && f_ok) begin
      if ($past(ready)) assert (!ready);
      assert (slot == !$past(slot));
      if ($past(restart)) begin
        assert (pc == $past(start_pc) && stall == 1 && sp == 0 && !z && !c && !to && !blocked);
        assert (r[f_ri] == 16'd0);
      end else if ($past(ready)) begin
        assert (pc == $past(pc_n) && z == $past(z_n) && c == $past(c_n) && to == $past(to_n));
        assert (stall == $past(stall_n));
        assert (blocked == !$past(done));
        if (!(($past(we_a) && $past(wa_i) == f_ri) || ($past(we_b) && $past(wb_i) == f_ri)))
          assert (r[f_ri] == f_rv);
        if ($past(we_a) && $past(wa_i) == f_ri) assert (r[f_ri] == $past(wa_v));
      end else begin
        assert (pc == $past(pc) && z == $past(z) && c == $past(c) && to == $past(to));
        assert (r[f_ri] == f_rv);
        assert (sp == $past(sp));
        assert (stall == (($past(stall) == 0) ? 0 : $past(stall) - 1'b1));
      end
    end
  end
`endif

endmodule
