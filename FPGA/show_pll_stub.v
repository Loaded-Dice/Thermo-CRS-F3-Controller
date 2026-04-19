// Minimal simulation stub for schematic export.
// Needed so Yosys can parse SB_PLL40_CORE without loading full iCE40 cell libs.
module SB_PLL40_CORE #(
    parameter FEEDBACK_PATH = "SIMPLE",
    parameter DIVR = 4'b0000,
    parameter DIVF = 7'b0000000,
    parameter DIVQ = 3'b000,
    parameter FILTER_RANGE = 3'b000
) (
    input  wire REFERENCECLK,
    output wire PLLOUTGLOBAL,
    input  wire RESETB,
    input  wire BYPASS
);
    assign PLLOUTGLOBAL = REFERENCECLK;
endmodule
