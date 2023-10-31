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

### Optional: Instruction/Data memory
* Instead of FSM microcode (or microcode) holding values for OP-codes, we instead implement an instruction and data cache.
- Instruction cache
  - 32 bit cache, 256 lines, 2 ways: 16kb size
- Data Cache
  - 



