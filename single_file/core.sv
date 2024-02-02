`include "decode.sv"
module RISCV (
  input wire clock,
  input wire rst,
  output wire [31:0] io_addr,
  output wire [31:0] io_rdata,
  output wire [31:0] io_wdata,
  output wire [31:0] io_wrf
);


