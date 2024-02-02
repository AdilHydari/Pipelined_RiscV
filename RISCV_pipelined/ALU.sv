`include "DEFS.sv"
module alu (
  input [4:0] alu_function,
  input signed [31:0] in1,
  input signed [31:0] in2,
  output logic result,
  output       zero_flag
);
//“M” Standard Extension for Integer Multiplication and Division
  `ifdef M_EXT // This is used to signify the M extension. This means that the FUNCT3 part of instruction decoding will be responsible for signifying what type of multiplication should be used
    logic [63:0] signed_multiply;
    logic [63:0] unsigned_multiply;
    logic [63:0] signed_unsigned_multiply;
  `endif
assign zero_flag = (result == 32'b0);

always_comb begin
  result = `ZERO;
  case(alu_function)
    `ALU_ADD: result = in1 + in2;
    `ALU_SUB: result = in1 - in2;
    `ALU_SLL: result = in1 << in2[4:0];
    `ALU_SRL: result = in1 >> in2[4:0];
    `ALU_SRA: result = in1 >>> in2[4:0]; //Sign extend
    `ALU_SEQ: result = {31'b0, in1 == in2}; //BEQ, sets bit according to in1 == in2, signed
    `ALU_SLT: result = {31'b0, in1 < in2}; //BLT, "", signed
    `ALU_SLTU: result = {31'b0, $unsigned(in1) < $unsigned(in2)}; //BLT, "", unsigned
    `ALU_XOR: result = in1 ^ in2; //XOR
    `ALU_OR: result = in1 | in2; //OR
    `ALU_AND: result = in1 & in2; //AND
    `ifdef M_EXT`
            `ALU_MUL:   result = signed_multiplication[31:0];
            `ALU_MULH:  result = signed_multiplication[63:32];
            `ALU_MULHSU:    result = signed_unsigned_multiplication[63:32];
            `ALU_MULHU: result = unsigned_multiplication[63:32];
            `ALU_DIV:
                if (in2 == `ZERO)
                    result = 32'b1;
                else if ((in1 == 32'h80000000) && (in2 == 32'b1))
                    result = 32'h80000000;
                else
                    result = in1 / in2;
            `ALU_DIVU:
                if (in2 == `ZERO)
                    result = 32'b1;
                else
                    result = $unsigned(in1) / $unsigned(in2);
            `ALU_REM:
                if (in2 == `ZERO)
                    result = in1;
                else if ((in1 == 32'h80000000) && (in2 == 32'b1))
                    result = `ZERO;
                else
                    result = in1 % in2;
            `ALU_REMU:
                if (in2 == `ZERO)
                    result = in1;
                else
                    result = $unsigned(in1) % $unsigned(in2);
    `endif
            default:
                result = `ZERO;
        endcase
    end
    
    `ifdef M_EXT
        always_comb begin
            signed_multiplication   = in1 * in2;
            unsigned_multiplication = $unsigned(in1) * $unsigned(in2);
            signed_unsigned_multiplication = $signed(in1) * $unsigned(in2);
        end
    `endif

endmodule
