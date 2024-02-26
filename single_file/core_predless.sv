`include "decode.sv"

`define VERBOSE
module core_predless(
  input logic clk,
  input logic reset,
  output [31:0] IO_memory_addr,
  output [31:0] IO_memory_read,
  output [31:0] IO_memory_write,
  output        IO_memory_writef
  );

   function isALUreg; input [31:0] I; isALUreg=(I[6:0]==7'b0110011); endfunction
   function isALUimm; input [31:0] I; isALUimm=(I[6:0]==7'b0010011); endfunction
   function isBranch; input [31:0] I; isBranch=(I[6:0]==7'b1100011); endfunction
   function isJALR;   input [31:0] I; isJALR  =(I[6:0]==7'b1100111); endfunction
   function isJAL;    input [31:0] I; isJAL   =(I[6:0]==7'b1101111); endfunction
   function isAUIPC;  input [31:0] I; isAUIPC =(I[6:0]==7'b0010111); endfunction
   function isLUI;    input [31:0] I; isLUI   =(I[6:0]==7'b0110111); endfunction
   function isLoad;   input [31:0] I; isLoad  =(I[6:0]==7'b0000011); endfunction
   function isStore;  input [31:0] I; isStore =(I[6:0]==7'b0100011); endfunction
   function isSYSTEM; input [31:0] I; isSYSTEM=(I[6:0]==7'b1110011); endfunction

   /* Register indices */
   function [4:0] rs1Id; input [31:0] I; rs1Id = I[19:15];      endfunction
   function [4:0] rs2Id; input [31:0] I; rs2Id = I[24:20];      endfunction
   function [4:0] shamt; input [31:0] I; shamt = I[24:20];      endfunction   
   function [4:0] rdId;  input [31:0] I; rdId  = I[11:7];       endfunction
   function [1:0] csrId; input [31:0] I; csrId = {I[27],I[21]}; endfunction

   /* funct3 and funct7 */
   function [2:0] funct3; input [31:0] I; funct3 = I[14:12]; endfunction
   function [6:0] funct7; input [31:0] I; funct7 = I[31:25]; endfunction      

   
   /* EBREAK and CSRRS instruction "recognizers" */
   function isEBREAK; 
      input [31:0] I; 
      isEBREAK = (isSYSTEM(I) && funct3(I) == 3'b000); 
   endfunction

   function isCSRRS; 
      input [31:0] I; 
      isCSRRS = (isSYSTEM(I) && funct3(I) == 3'b010); 
   endfunction
   
   /* The 5 immediate formats */
   function [31:0] Uimm; 
      input [31:0] I; 
      Uimm={I[31:12],{12{1'b0}}}; 
   endfunction
   
   function [31:0] Iimm; 
      input [31:0] I; 
      Iimm={{21{I[31]}},I[30:20]};
   endfunction
   
   function [31:0] Simm; 
      input [31:0] I; 
      Simm={{21{I[31]}},I[30:25],I[11:7]};
   endfunction

   function [31:0] Bimm;
      input [31:0] I;
      Bimm = {{20{I[31]}},I[7],I[30:25],I[11:8],1'b0};
   endfunction 

   function [31:0] Jimm;
      input [31:0] I;
      Jimm = {{12{I[31]}},I[19:12],I[20],I[30:21],1'b0};      
   endfunction

   function writesRd;
      input [31:0] I;
      writesRd = !isStore(I) && !isBranch(I);
   endfunction

   function readsRs1;
      input [31:0] I;
      readsRs1 = !(isJAL(I) || isAUIPC(I) || isLUI(I));
   endfunction

   function readsRs2;
      input [31:0] I;
      readsRs2 = isALUreg(I) || isBranch(I) || isStore(I);
   endfunction

  //CSR performance logging + cycle count
 
   reg [63:0] cycle;   
   reg [63:0] instret;

   always @(posedge clk) begin
      cycle <= !reset ? 0 : cycle + 1;
   end

   wire DE_Flush;
   wire EX_Flush;
   
   wire IF_Stall;
   wire DE_Stall;

   wire halt; // Halt execution (on ebreak)
   
/******************************************************************************/

   localparam NOOP = 32'b0000000_00000_00000_000_00000_0110011;
   
                      /***  F: Instruction fetch ***/   

   reg  [31:0] 	  F_PC;

   /** These two signals come from the Execute stage **/
   wire [31:0] 	  jumpOrBranchAddress;
   wire 	  jumpOrBranch;

   reg [31:0] PROGROM[0:16383]; // 16384 4-bytes words  
                                // 64 Kb of program ROM 
   initial begin
      $readmemh("PROGROM.hex",PROGROM);
   end

   always @(posedge clk) begin

      if(!IF_Stall) begin
	 FD_instr <= PROGROM[F_PC[15:2]]; 
	 FD_PC    <= F_PC;
	 F_PC     <= F_PC+4;
      end

      if(jumpOrBranch) begin
	 F_PC     <= jumpOrBranchAddress;
      end

      // Cannot write NOP to FD_instr, because
      // whenever a BRAM read is involved, do
      // nothing else than sending the result
      // to a reg.
      FD_nop <= DE_Flush | !reset;
      
      if(!reset) begin
	 F_PC <= 0;
      end
   end
   
/******************************************************************************/
   reg [31:0] FD_PC;   
   reg [31:0] FD_instr;
   reg        FD_nop;
/******************************************************************************/

                     /*** D: Instruction decode ***/

   /** These three signals come from the Writeback stage **/
   wire        wbEnable;
   wire [31:0] wbData;
   wire [4:0]  wbRdId;

   reg [31:0] RegisterBank [0:31];
   always @(posedge clk) begin

      if(!DE_Stall) begin
	 DE_PC    <= FD_PC;
	 DE_instr <= (EX_Flush | FD_nop) ? NOOP : FD_instr;
      end
      
      if(EX_Flush) begin
	 DE_instr <= NOOP;
      end
      
      DE_rs1 <= RegisterBank[rs1Id(FD_instr)];
      DE_rs2 <= RegisterBank[rs2Id(FD_instr)];
      
      if(wbEnable) begin
	 RegisterBank[wbRdId] <= wbData;
      end
   end
   
/******************************************************************************/
   reg [31:0] DE_PC;
   reg [31:0] DE_instr;
   reg [31:0] DE_rs1;
   reg [31:0] DE_rs2;
   
 
                     /*** E: Execute ***/

   /*********** the ALU *************************************************/

   wire [31:0] E_aluIn1 = DE_rs1;
   
   wire [31:0] E_aluIn2 = 
         (isALUreg(DE_instr) | isBranch(DE_instr)) ? DE_rs2 : Iimm(DE_instr);
   
   wire [4:0]  E_shamt  = isALUreg(DE_instr) ? DE_rs2[4:0] : shamt(DE_instr); 

   wire E_minus = DE_instr[30] & isALUreg(DE_instr);
   wire E_arith_shift = DE_instr[30];
   
   // The adder is used by both arithmetic instructions and JALR.
   wire [31:0] E_aluPlus = E_aluIn1 + E_aluIn2;

   // Use a single 33 bits subtract to do subtraction and all comparisons
   // (trick borrowed from swapforth/J1)
   wire [32:0] E_aluMinus = {1'b1, ~E_aluIn2} + {1'b0,E_aluIn1} + 33'b1;
   wire        E_LT  = 
                 (E_aluIn1[31] ^ E_aluIn2[31]) ? E_aluIn1[31] : E_aluMinus[32];
   wire        E_LTU = E_aluMinus[32];
   wire        E_EQ  = (E_aluMinus[31:0] == 0);

   // Flip a 32 bit word. Used by the shifter (a single shifter for
   // left and right shifts, saves silicium !)
   function [31:0] flip32;
      input [31:0] x;
      flip32 = {x[ 0], x[ 1], x[ 2], x[ 3], x[ 4], x[ 5], x[ 6], x[ 7], 
		x[ 8], x[ 9], x[10], x[11], x[12], x[13], x[14], x[15], 
		x[16], x[17], x[18], x[19], x[20], x[21], x[22], x[23],
		x[24], x[25], x[26], x[27], x[28], x[29], x[30], x[31]};
   endfunction

   wire [31:0] E_shifter_in = 
                      (funct3(DE_instr)==3'b001) ? flip32(E_aluIn1) : E_aluIn1;
   
   /* verilator lint_off WIDTH */
   wire [31:0] E_shifter = 
       $signed({E_arith_shift & E_aluIn1[31], E_shifter_in}) >>> E_aluIn2[4:0];
   /* verilator lint_on WIDTH */

   wire [31:0] E_leftshift = flip32(E_shifter);

   reg [31:0] E_aluOut;
   always @(*) begin
      case(funct3(DE_instr))
	3'b000: E_aluOut = E_minus ? E_aluMinus[31:0] : E_aluPlus;
	3'b001: E_aluOut = E_leftshift;
	3'b010: E_aluOut = {31'b0, E_LT};
	3'b011: E_aluOut = {31'b0, E_LTU};
	3'b100: E_aluOut = E_aluIn1 ^ E_aluIn2;
	3'b101: E_aluOut = E_shifter;
	3'b110: E_aluOut = E_aluIn1 | E_aluIn2;
	3'b111: E_aluOut = E_aluIn1 & E_aluIn2;
      endcase
   end
   
   /*********** Branch, JAL, JALR ***********************************/

   reg E_takeBranch;
   always @(*) begin
      case (funct3(DE_instr))
	3'b000: E_takeBranch = E_EQ;
	3'b001: E_takeBranch = !E_EQ;
	3'b100: E_takeBranch = E_LT;
	3'b101: E_takeBranch = !E_LT;
	3'b110: E_takeBranch = E_LTU;
	3'b111: E_takeBranch = !E_LTU;
	default: E_takeBranch = 1'b0;
      endcase 
   end
   
   wire E_JumpOrBranch = (
         isJAL(DE_instr)  || 
         isJALR(DE_instr) || 
         (isBranch(DE_instr) && E_takeBranch)
   );

   wire [31:0] E_JumpOrBranchAddr =
	isBranch(DE_instr) ? DE_PC + Bimm(DE_instr) :
	isJAL(DE_instr)    ? DE_PC + Jimm(DE_instr) :
	/* JALR */           {E_aluPlus[31:1],1'b0} ;

   wire [31:0] E_result = 
	(isJAL(DE_instr) | isJALR(DE_instr)) ? DE_PC+4                :
	isLUI(DE_instr)                      ? Uimm(DE_instr)         :
	isAUIPC(DE_instr)                    ? DE_PC + Uimm(DE_instr) : 
        E_aluOut                                                      ;
	
   /**************************************************************/
   
   always @(posedge clk) begin
      EM_PC      <= DE_PC;
      EM_instr   <= DE_instr;
      EM_rs2     <= DE_rs2;
      EM_Eresult <= E_result;
      EM_addr    <= isStore(DE_instr) ? DE_rs1 + Simm(DE_instr) : 
                                        DE_rs1 + Iimm(DE_instr) ;
   end

   assign halt = reset & isEBREAK(DE_instr);
   
/******************************************************************************/
   reg [31:0] EM_PC;
   reg [31:0] EM_instr;
   reg [31:0] EM_rs2;
   reg [31:0] EM_Eresult;
   reg [31:0] EM_addr;
/******************************************************************************/

                     /*** M: Memory ***/

   wire [2:0] M_funct3 = funct3(EM_instr);
   wire M_isB = (M_funct3[1:0] == 2'b00);
   wire M_isH = (M_funct3[1:0] == 2'b01);

   /*************** STORE **************************/

   wire [31:0] M_STORE_data;
   assign M_STORE_data[ 7: 0] = EM_rs2[7:0];
   assign M_STORE_data[15: 8] = EM_addr[0] ? EM_rs2[7:0]  : EM_rs2[15: 8] ;
   assign M_STORE_data[23:16] = EM_addr[1] ? EM_rs2[7:0]  : EM_rs2[23:16] ;
   assign M_STORE_data[31:24] = EM_addr[0] ? EM_rs2[7:0]  :
			        EM_addr[1] ? EM_rs2[15:8] : EM_rs2[31:24] ;

   // The memory write mask:
   //    1111                     if writing a word
   //    0011 or 1100             if writing a halfword
   //                                (depending on EM_addr[1])
   //    0001, 0010, 0100 or 1000 if writing a byte
   //                                (depending on EM_addr[1:0])

   wire [3:0] M_STORE_wmask = M_isB ?
	                     (EM_addr[1] ?
		                (EM_addr[0] ? 4'b1000 : 4'b0100) :
		                (EM_addr[0] ? 4'b0010 : 4'b0001)
                             ) :
	                     M_isH ? (EM_addr[1] ? 4'b1100 : 4'b0011) :
                                      4'b1111 ;


   wire  M_isIO         = EM_addr[22];
   wire  M_isRAM        = !M_isIO;

   assign IO_memory_addr  = EM_addr;
   assign IO_memory_writef= isStore(EM_instr) && M_isIO; // && M_STORE_wmask[0];
   assign IO_memory_write = EM_rs2;

   wire [3:0] M_wmask = {4{isStore(EM_instr) & M_isRAM}} & M_STORE_wmask;
   
   reg [31:0] DATARAM [0:16383]; // 16384 4-bytes words 
                                 // 64 Kb of data RAM in total
   wire [13:0] M_word_addr = EM_addr[15:2];
   
   always @(posedge clk) begin
      MW_Mdata <= DATARAM[M_word_addr];
      if(M_wmask[0]) DATARAM[M_word_addr][ 7:0 ] <= M_STORE_data[ 7:0 ];
      if(M_wmask[1]) DATARAM[M_word_addr][15:8 ] <= M_STORE_data[15:8 ];
      if(M_wmask[2]) DATARAM[M_word_addr][23:16] <= M_STORE_data[23:16];
      if(M_wmask[3]) DATARAM[M_word_addr][31:24] <= M_STORE_data[31:24]; 
   end
   
   initial begin
      $readmemh("DATARAM.hex",DATARAM);
   end
   
   always @(posedge clk) begin
      MW_PC        <= EM_PC;
      MW_instr     <= EM_instr;
      MW_Eresult   <= EM_Eresult;
      MW_IOresult  <= IO_memory_read;
      MW_addr      <= EM_addr;
      case(csrId(EM_instr)) 
	2'b00: MW_CSRresult = cycle[31:0];
	2'b10: MW_CSRresult = cycle[63:32];
	2'b01: MW_CSRresult = instret[31:0];
	2'b11: MW_CSRresult = instret[63:32];	 
      endcase 
      if(!reset) begin
	 instret <= 0;
      end else if(MW_instr != NOOP) begin
	 instret <= instret + 1;
      end
   end

/******************************************************************************/
   reg [31:0] MW_PC; 
   reg [31:0] MW_instr; 
   reg [31:0] MW_Eresult;
   reg [31:0] MW_addr;
   reg [31:0] MW_Mdata;
   reg [31:0] MW_IOresult;
   reg [31:0] MW_CSRresult;
/******************************************************************************/

                     /*** W: WriteBack ***/
		     
   wire [2:0] W_funct3 = funct3(MW_instr);
   wire W_isB = (W_funct3[1:0] == 2'b00);
   wire W_isH = (W_funct3[1:0] == 2'b01);
   wire W_sext = !W_funct3[2];		     
   wire W_isIO = MW_addr[22];

   /*************** LOAD ****************************/
   
   wire [15:0] W_LOAD_H=MW_addr[1] ? MW_Mdata[31:16]: MW_Mdata[15:0];
   wire  [7:0] W_LOAD_B=MW_addr[0] ? W_LOAD_H[15:8] : W_LOAD_H[7:0];
   wire        W_LOAD_sign=W_sext & (W_isB ? W_LOAD_B[7] : W_LOAD_H[15]);

   wire [31:0] W_Mresult = W_isB ? {{24{W_LOAD_sign}},W_LOAD_B} :
	                   W_isH ? {{16{W_LOAD_sign}},W_LOAD_H} :
                                                      MW_Mdata ;
   
   assign wbData = 
	       isLoad(MW_instr)  ? (W_isIO ? MW_IOresult : W_Mresult) :
	       isCSRRS(MW_instr) ? MW_CSRresult :
	       MW_Eresult;

   assign wbEnable = writesRd(MW_instr) && rdId(MW_instr) != 0;
   assign wbRdId = rdId(MW_instr);
   
/******************************************************************************/
   assign jumpOrBranchAddress = E_JumpOrBranchAddr;
   assign jumpOrBranch        = E_JumpOrBranch;

   wire rs1Hazard = !FD_nop && readsRs1(FD_instr) && rs1Id(FD_instr) != 0 && (
               (writesRd(DE_instr) && rs1Id(FD_instr) == rdId(DE_instr)) ||
               (writesRd(EM_instr) && rs1Id(FD_instr) == rdId(EM_instr)) ||
	       (writesRd(MW_instr) && rs1Id(FD_instr) == rdId(MW_instr)) ) ;

   wire rs2Hazard = !FD_nop && readsRs2(FD_instr) && rs2Id(FD_instr) != 0 && (
               (writesRd(DE_instr) && rs2Id(FD_instr) == rdId(DE_instr)) ||
               (writesRd(EM_instr) && rs2Id(FD_instr) == rdId(EM_instr)) ||
	       (writesRd(MW_instr) && rs2Id(FD_instr) == rdId(MW_instr)) ) ;
   
   wire dataHazard = rs1Hazard || rs2Hazard;
   
   assign IF_Stall = dataHazard | halt;
   assign DE_Stall = dataHazard | halt;
   
   assign DE_Flush = E_JumpOrBranch;
   assign EX_Flush = E_JumpOrBranch | dataHazard;

/******************************************************************************/

`ifdef BENCH
   always @(posedge clk) begin
      if(halt) $finish();
   end
`endif

`ifdef VERBOSE   
   always @(posedge clk) begin
      if(reset) begin
	 $write("[W] PC=%h ", MW_PC);
	 $write("     ");
	 riscv_disasm(MW_instr,MW_PC);
	 if(wbEnable) $write("    x%0d <- 0x%0h",rdId(MW_instr),wbData);
	 $write("\n");

	 $write("[M] PC=%h ", EM_PC);
	 $write("     ");	 
	 riscv_disasm(EM_instr,EM_PC);
	 $write("\n");

	 $write("[E] PC=%h ", DE_PC);
	 $write("     ");	 
	 riscv_disasm(DE_instr,DE_PC);
	 if(DE_instr != NOOP) begin
	    $write("  rs1=0x%h  rs2=0x%h  ",DE_rs1, DE_rs2);
	 end
	 $write("\n");

	 $write("[D] PC=%h ", FD_PC);
	 $write("[%s%s] ",rs1Hazard?"*":" ",rs2Hazard?"*":" ");	 
	 riscv_disasm(FD_nop ? NOOP : FD_instr,FD_PC);
	 $write("\n");

	 $write("[F] PC=%h ", F_PC); 
	 if(jumpOrBranch) $write(" PC <- 0x%0h",jumpOrBranchAddress);
	 $write("\n");
	 
	 $display("");
      end
   end
`endif

/******************************************************************************/
   
endmodule


     
  






