# 5 stage pipelined version of Single Cycle RISC-V processor

![Pipeline](https://github.com/AdilHydari/Pipelined_RiscV/blob/main/image_source/Pipeline_structure.png)

  - 32 bit
  - IF: Instruction Fetch
  - ID: Instruction Decode
  - EX: Execute
  - MEM: Memory
  - WB: Write Back
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

## Forwarding Unit
- The reference does not include this. The main purpose of a forwarding unit is to aid in the data hazards that can occur in a pipelined processor. Data hazards are detected when the source register contains the same instruction as the destination register of the previous instruction. 
  - [Pipeline Stall Wikipedia](en.wikipedia.org/wiki/Pipeline_stall)

### Stall unit
- This plays into a forwarding unit, a stall happens when the CLU determines if the decoded instruction reads from a register to which the currently instruction writes. If this holds a stall, or a *bubble* occurs. 
  - As an example: In a risc pipeline, if we have data after the MEM stage of the pipeline that is required by a second instruction in the EX stage that is delayed by a clock cycle. Since we can only read forwards and not backwards in clock cycle, we need to insert a stall in the second instruction. By doing this, we can pass the data in the first instruction in MEM forwards to the EX stage of the second instruction's pipeline. 
![RISC-stall](https://github.com/AdilHydari/Pipelined_RiscV/blob/main/image_source/RISC_stall.png)
## Top level (Processor)
- 




