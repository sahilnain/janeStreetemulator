// Black box of the IHP SRAM macro RM_IHPSG13_1P_256x8_c3_bm_bist (IHP-Open-PDK,
// libs.ref/sg13g2_sram), for synthesis: the flow places the hard macro from
// src/config.json MACROS. Simulation uses the PDK's behavioural model instead
// (test/models, with FUNCTIONAL defined).
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNDRIVEN */
(* blackbox *)
module RM_IHPSG13_1P_256x8_c3_bm_bist (
    input A_CLK,
    input A_MEN,
    input A_WEN,
    input A_REN,
    input [7:0] A_ADDR,
    input [7:0] A_DIN,
    input A_DLY,
    output [7:0] A_DOUT,
    input [7:0] A_BM,
    input A_BIST_CLK,
    input A_BIST_EN,
    input A_BIST_MEN,
    input A_BIST_WEN,
    input A_BIST_REN,
    input [7:0] A_BIST_ADDR,
    input [7:0] A_BIST_DIN,
    input [7:0] A_BIST_BM
);
endmodule
/* verilator lint_on UNDRIVEN */
/* verilator lint_on UNUSEDSIGNAL */
