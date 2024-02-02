/*
************************************************************
Address decoding for I, S, U, Uraw (AUIPC, LUI), B and J type IMM instructions
ALUreg:
  ALUREG consists of an arithmetic and logical unit (ALU) and a register file.
  https://usercontent.irccloud-cdn.com/file/oI4aJRi8/ALUREG.png
  ALUreg does not support divide inst. instead, it allows for single cycle I and Mult. inst.
ALUimm:
  The selector signal for input b of the ALU; uses IMM for immediate addr decoding.

BRANCH: 
  Reference: RVG opcodes(images)
  B Type instruction; comparison of 2 registers

JALR:
  Indirect jump; JUMP and LINK REGISTER; I-type encoding
  JALR used to enable a two-instruction sequence to jump anywhere in a 32-bit absolute address range

JAL: 
  J-type encoding; Plain old unconditional Jumps 

AUIPC:
  U-type encoding; forms a 32-bit offset from the 20-bit U-immediate, filling in the lowest 12 bits with zeros, adds this offset to the address of the AUIPC instruction, then places the result in register rd

LUI:
  U-type encoding; places the U-immediate value in the top 20 bits of the destination register rd, filling in the lowest 12 bits with zeros.

LOAD:
  What it sounds like.

STORE:
  What it sounds like.

SYSTEM:
  I-type encoding; SYSTEM instructions are used to access system functionality that might require privileged access; i.e. (some) CSRs and potential privileged instructions.

***************************************************************************************************
   5'b01100 | ALUreg  | rd <- rs1 OP rs2   
   5'b00100 | ALUimm  | rd <- rs1 OP Iimm
   5'b11000 | Branch  | if(rs1 OP rs2) PC<-PC+Bimm
   5'b11001 | JALR    | rd <- PC+4; PC<-rs1+Iimm
   5'b11011 | JAL     | rd <- PC+4; PC<-PC+Jimm
   5'b00101 | AUIPC   | rd <- PC + Uimm
   5'b01101 | LUI     | rd <- Uimm   
   5'b00000 | Load    | rd <- mem[rs1+Iimm]
   5'b01000 | Store   | mem[rs1+Simm] <- rs2
   5'b11100 | SYSTEM  | opcode
***************************************************************************************************
Created according to chapter 25 of the RISC-V spec documentation.
*/
// function: https://www.chipverify.com/systemverilog/systemverilog-functions
// didn't know these existed
/* 
 Instr. 	    Description 	                            Immediate value encoding
R-type 	  register-register ALU ops. 	                            None
I-type 	  register-immediate integer ALU ops and JALR. 	 12 bits, sign expansion
S-type 	  store 	                                       12 bits, sign expansion
B-type 	  branch 	                                       12 bits, sign expansion, upper [31:1]
U-type 	  LUI,AUIPC 	                                   20 bits, upper 31:12 (bits [11:0] are 0)
J-type 	  JAL 	                                         12 bits, sign expansion, upper [31:1]
*/

function signed [31:0] riscv_dis_I_imm;
  input [31:0] instr;
  riscv_dis_I_imm = {{21{instr[31]}}, instr[30:20]};
//{21{instr[31]}}: performs a sign extension on bit 32, 21 times, this converts a 12-bits signed quantity into a 32-bits one.
//instr[30:20]:  This selects bits 20 through 30 from the instr variable, which represents a portion of the immediate value encoded in an I-type. 
// {{21{instr[31]}}, instr[30:20]}: This operation concatenates the 21 replicated sign bits with the 11 bits selected from the instr variable. The upper 21 bits are all copies of the sign bit, and the lower 11 bits are the actual immediate value from the instruction.
endfunction 

function signed [31:0] riscv_dis_S_imm;
  input [31:0] instr;
  riscv_dis_S_imm = {{21{instr[31]}}, instr[30:25],instr[11:7]}; 
// Sign extend
//For S-type instructions, [11:7] also form part of the immediate value.
//The bits [30:25] are also a part of the immediate value.
//Concatenates all three to form S-type
endfunction

function signed [31:0] riscv_dis_U_imm_only;
  input [31:0] instr;
  riscv_dis_U_imm_only = {instr[31:12]};
//Used for LUI and AUIPC
//i.e. AUIPC: You need rd, the 20 bit immediate, then PC+U_immediate (shifted left by 12)  would give the address of the AUIPC target
endfunction

function signed [31:0] riscv_dis_U_imm;
  input [31:0] instr;
  riscv_dis_U_imm = {instr[31],instr[30:12],{12{1'b0}}};
endfunction

function [31:0] riscv_dis_B_imm;
  input [31:0] instr;
  riscv_dis_B_imm = {
	  {20{instr[31]}},instr[7],instr[30:25],instr[11:8],1'b0};
endfunction
/*
{20{instr[31]}}: Takes the 32nd bit of the instruction, which is the sign bit of the B-immediate, and replicates it 20 times to sign extend it.

instr[7]: This bit is part of the immediate value for B-type instructions

instr[30:25]: These bits are also part of the immediate value

instr[11:8]: These four bits are the next part of the immediate value..

1'b0: Finally, a single '0' bit is appended to the least significant bit of the immediate. This is because B-type immediate values are word-aligned, meaning they are always a multiple of 2, so theleast significant bit is always 0.

*/

function [31:0] riscv_dis_J_imm;
  input [31:0] instr;
  riscv_dis_J_imm = {
          {12{instr[31]}},instr[19:12],instr[20],instr[30:21],1'b0};
endfunction
//Similar to B-type instruction

//
task riscv_dis;
  input instr[31:0];
  input PC[31:0];
  begin
    case(instr[6:0]):
      7'b0110011: begin
        if(instr[31:7] == 0) begin
          $write("nop");
        end else begin

