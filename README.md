# 5 stage pipelined version of Single Cycle RISC-V processor

![Pipeline](https://github.com/AdilHydari/Pipelined_RiscV/blob/main/image_source/Pipeline_structure.png)

  - 32 bit
  - IF: Instruction Fetch
  - ID: Instruction Decode
  - EX: Execute
  - MEM: Memory
  - WB: Write Back
  - Pipeline registers
    - IF/ID
    - ID/EX
    - EX/MEM
    - MEM/WB

## Instruc mem and Data mem
- OP-codes are fetched during the IF stage from the instruction memory
- Data memory is used for the Data-path block in order to hold data between different pipeline stages (ie holding the value of the EX/MEM register)

### Instruction/Data memory
* Optional: Instead of FSM microcode (or microcode) holding values for OP-codes, we instead implement an instruction and data cache.
- Instruction Memory
  - 32 bit cache, 256 lines, 2 ways: 16kb size
- Data memory
  - Essentially is the DRAM of the system, used to store temp files, the reference design I have does not have that, so I must find a way to implement it myself. Most likely, the forwarding unit will handle all the memory processes for the DRAM, as I expect it to. 
## Register file
- There needs to be a register file that is able to read and write to registers.
  - Register file always outputs the contents of the register corresponding to read register numbers specified. Reading a register is not dependent on any other signals.
  - Register Write: Register writes are controlled by a control signal (Control Unit) RegWrite. The write should happen if RegWrite signal is 1 and on the posedge the clock.

## Data/Control hazards
- Data hazards: an instruction is unable to execute in the planned cycle because it is dependent on some data that has not yet been committed.
- Control hazards: we do not know which instruction needs to be executed next.
- Forwarding (or bypassing): the needed data is forwarded as soon as possible to the instruction which depends on it.
- Stalling: the dependent instruction is “pushed back” for one or more clock cycles. Alternatively, you can think of stalling as the execution of a noop for one or more cycles.

## Forwarding Unit
- The reference does not include this. The main purpose of a forwarding unit is to aid speeding up the data hazards that can occur in a pipelined processor. Data hazards are detected when the source register contains the same instruction as the destination register of the previous instruction. 
  - A forwarding unit can be used to circumvent the cycle loss caused by a stall. A forward allows other instructions to read ALU results directly from the pipeline registers rather than having to read directly from the register file. 
  - [Pipeline Stall Wikipedia](en.wikipedia.org/wiki/Pipeline_stall)
  - [Helpful powerpoint](https://courses.cs.washington.edu/courses/cse378/09wi/lectures/lec12.pdf)

### Stall unit
- This plays into a forwarding unit, a stall happens when the CLU determines if the decoded instruction reads from a register to which the currently instruction writes. If this holds a stall, or a *bubble* occurs. 
  - As an example: In a risc pipeline, if we have data after the MEM stage of the pipeline that is required by a second instruction in the EX stage that is delayed by a clock cycle. Since we can only read forwards and not backwards in clock cycle, we need to insert a stall in the second instruction. By doing this, we can pass the data in the first instruction in MEM forwards to the EX stage of the second instruction's pipeline. 
![RISC-stall](https://github.com/AdilHydari/Pipelined_RiscV/blob/main/image_source/RISC_stall.png)
![Helpful powerpoint](https://www.cs.fsu.edu/~zwang/files/cda3101/Fall2017/Lecture9_cda3101.pdf)

## Control Unit
- A control unit is what is sounds like: it controls aspects of the Pipeline's datapath. (ie RegWrite)
![Control Unit](https://github.com/AdilHydari/Pipelined_RiscV/blob/main/image_source/Control_signal.png)
![Control Unit desc](https://github.com/AdilHydari/Pipelined_RiscV/blob/main/image_source/Control_signal_desc.png)

## ALU 
- The ALU controls all core arithmetic and logical processes for the instruction. It can add registers together, and performs all AND, OR, NOT, etc. operations. It also handles carries in addition by using logical shifts. 
  -  Carry: set to 1 if there is a carry in the most-significant bit which we could not output in the given number of bits (32 bits, above).
  - Overflow: set to 1 of the input operands have the same sign, and the result has a different sign. For example, if the sum of two positive numbers yields a negative result, then an overflow occurs. 

## Datapath
- Instruction fields and data generally move from left-to-right as they progress through
each stage.
The two exceptions are:
  - The WB stage places the result back into the register file in the middle of the
datapath leads to data hazards.
  - The selection of the next value of the PC – either the incremented PC or the branch address leads to control hazards.
  - [Helpful powerpoint](https://www.cs.fsu.edu/~zwang/files/cda3101/Fall2017/Lecture8_cda3101.pdf)
  

- Load Word example:
  - Instruction Fetch (IF)
    - The instruction is read from memory using the contents of PC and placed in the IF/ID register.
    - The PC address is incremented by 4 and written back to the PC register, as well as placed in the IF/ID register in case the instruction needs it later
  - Instruction Decode and Register File Read (ID):
    - The registers $rs and $rt are read from the register file and stored in the ID/EX pipeline register. Remember, we don’t know what the instruction is yet.
    - The 16-bit immediate field is sign-extended to 32-bits and stored in the ID/EX pipeline register.
    - The PC+4 value is copied from the IF/ID register into the ID/EX register in case the instruction needs it later.
  - Execute or Address Calculation (EX)
    - From the ID/EX pipeline register, take the contents of $rs and the sign-extended immediate field as inputs to the ALU, which performs an add operation. The sum is placed in the EX/MEM pipeline register.
  - Memory Access (MEM)
    - Take the address stored in the EX/MEM pipeline register and use it to access data memory. The data read from memory is stored in the MEM/WB pipeline register.
  - Write Back (WB)
    - Read the data from the MEM/WB register and write it back to the register file in the middle of the datapath




## Top level (Processor)
- 




