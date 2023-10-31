/*
The ALU should be divided into 2 parts, the adder as well as a bit shift
ASL is the shift arithmetic left 1 bit
LSR is the logical shift left 
These values must be stored in memory or within an accumulator register (A)
The use of the ASL and LSR is with binary addition, when we have a case of 1+1, we will generate a carry; this means that there needs to be a binary shift left in order to increment the binary value */ 


module alu(
  input [7:0] register, //Register input
  input [7:0] memory,
  input [4:0] operation_sel,
  input CI, //Carry in bit
  input SI, //shift in bit
  output CO, //carry out bit
  output reg [7:0] OUT    // data out
);
/*
  op   function
  ---  --------
  000   R | M      OR 
  001   R & M      AND
  010   R ^ M      EOR
  011   R + M      ADC
  100   R + 0      R or INC depending on CI
  101   R - 1      R or DEC depending on CI
  110   R - M      SBC/CMP
  111  ~R & M      TRB
  As this is a 8 bit processor, we cannot use a half-adder since that will only allow 2 bit adding, instead we must implement a full adder; using a carry in bit in order to create this. 
    https://c74project.com/card-b-alu-cu/
    Todo: there needs to be a half carry as well; since the orginal 65c02 included a high/low nibble adder, each being 4 bit.
      This means that the first low nibble adder must be a standard adder, but the high nibble must be a carry-skip adder, meaning that it "skips" the first 4 bi      t register in order to reduce propagation delay that would incur if 1 standard adder handled the full 8 bit register.

      The Skip Adder consists of two adder ICs working in parallel to process two results concurrently for the high-nibble, one which assumes an input carry set to zero (ADR.HI.CLO) and the other with an input carry set to one (ADR.HI.CHI). The low-nibble carry (ADRLC) selects between these two results using the SKIP.ADR multiplexer. This deign is much faster than a traditional ripple-carry design since the high-nibble is computed concurrently with the low-nibble, rather than having to wait for the low-nibble carry. The low-nibble carry (ADRLC) is also used to select between the carry outputs of the high-nibble, CLO.C and CHI.C, to generate the Skip Adder’s output Carry. This is done by the C.OUT Multiplexer. 

CI must exist in order to recurse through carry-chain logic.

THIS IS VERY ARCHIAC IMPL, needs to be redone once op-codes and DBus gets spec

*/

/*
 * 8 bit inverted version of M and R (to avoid creating
 * 9 bit expressions)(when ~M is used in the expression, 
 */
wire [7:0] N = ~memory;

reg [8:0] temp_reg;

always @(*)
  case( operation_sel[2:0] )
        3'b000: temp =  register |  memory + CI;
        3'b001: temp =  register &  memory + CI;
        3'b010: temp =  register ^  memory + CI;
        3'b011: temp =  register +  memory + CI;
        3'b100: temp =  register +  0 + CI;
        3'b101: temp =  register + ~0 + CI; 
        3'b110: temp =  register + N + CI;
        3'b111: temp = ~register &  memory + CI;
    endcase


