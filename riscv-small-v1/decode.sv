// Enum for opcodes
typedef enum logic [6:0] {
    OPCODE_LUI     = 7'b0110111,
    OPCODE_AUIPC   = 7'b0010111,
    OPCODE_JAL     = 7'b1101111,
    OPCODE_JALR    = 7'b1100111,
    OPCODE_BRANCH  = 7'b1100011,
    OPCODE_LOAD    = 7'b0000011,
    OPCODE_STORE   = 7'b0100011,
    OPCODE_ALUIMM  = 7'b0010011,
    OPCODE_ALUREG  = 7'b0110011,
    OPCODE_SYSTEM  = 7'b1110011
} opcode_e;

// Enum for funct3 - Used in ALU operations, loads, stores, branches
typedef enum logic [2:0] {
    FUNCT3_ADD_SUB = 3'b000,
    FUNCT3_SLL     = 3'b001,
    FUNCT3_SLT     = 3'b010,
    FUNCT3_SLTU    = 3'b011,
    FUNCT3_XOR     = 3'b100,
    FUNCT3_SRX     = 3'b101, // SRL or SRA
    FUNCT3_OR      = 3'b110,
    FUNCT3_AND     = 3'b111,
    // Branch-specific funct3 codes
    FUNCT3_BEQ     = 3'b000,
    FUNCT3_BNE     = 3'b001,
    FUNCT3_BLT     = 3'b100,
    FUNCT3_BGE     = 3'b101,
    FUNCT3_BLTU    = 3'b110,
    FUNCT3_BGEU    = 3'b111,
    // Load and Store specific funct3 codes
    FUNCT3_LB      = 3'b000,
    FUNCT3_LH      = 3'b001,
    FUNCT3_LW      = 3'b010,
    FUNCT3_LBU     = 3'b100,
    FUNCT3_LHU     = 3'b101,
    FUNCT3_SB      = 3'b000,
    FUNCT3_SH      = 3'b001,
    FUNCT3_SW      = 3'b010,
    //RV32M
    FUNCT3_MUL     = 3'b000,
    FUNCT3_MULH    = 3'b001,
    FUNCT3_MULHSU  = 3'b010,
    FUNT3_MULHU    = 3'b011,
    FUNCT3_DIV     = 3'b100,
    FUNCT3_DIVU    = 3'b101,
    FUNCT3_REM     = 3'b110,
    FUNCT3_REMU    = 3'b111
} funct3_e;

// Enum for funct7 - Used in ALU operations for R-type instructions
typedef enum logic [6:0] {
    FUNCT7_ADD  = 7'b0000000,
    FUNCT7_SUB  = 7'b0100000,
    FUNCT7_SLL  = 7'b0000000,
    FUNCT7_SLT  = 7'b0000000,
    FUNCT7_SLTU = 7'b0000000,
    FUNCT7_XOR  = 7'b0000000,
    FUNCT7_SRL  = 7'b0000000,
    FUNCT7_SRA  = 7'b0100000,
    FUNCT7_OR   = 7'b0000000,
    FUNCT7_AND  = 7'b0000000,
    // RV32M extension funct7 codes
    FUNCT7_MUL    = 7'b0000001,
    FUNCT7_MULH   = 7'b0000001,
    FUNCT7_MULHSU = 7'b0000001,
    FUNCT7_MULHU  = 7'b0000001,
    FUNCT7_DIV    = 7'b0000001,
    FUNCT7_DIVU   = 7'b0000001,
    FUNCT7_REM    = 7'b0000001,
    FUNCT7_REMU   = 7'b0000001
} funct7_e;

// Enum for CSR registers and SYSTEM instructions
typedef enum logic [11:0] {
    CSR_RDCYCLE    = 12'hC00,
    CSR_RDCYCLEH   = 12'hC80,
    CSR_RDINSTRET  = 12'hC02,
    CSR_RDINSTRETH = 12'hC82,
    // Add more CSR codes as required
    SYSTEM_ECALL    = 12'h000,
    SYSTEM_EBREAK   = 12'h001
    // Add more SYSTEM opcodes as needed
} csr_e;

// The above definitions allow for clear and manageable instruction decoding in your design.

