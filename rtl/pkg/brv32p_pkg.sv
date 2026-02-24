// ============================================================================
// brv32p_pkg.sv — Shared definitions for BRV32P pipelined RISC-V MCU
// ============================================================================
// RV32IMC, 5-stage pipeline, AXI4-Lite bus, I$/D$ caches
// ============================================================================
package brv32p_pkg;

  // ── Instruction Opcodes ──────────────────────────────────────────────
  typedef enum logic [6:0] {
    OP_LUI      = 7'b0110111,
    OP_AUIPC    = 7'b0010111,
    OP_JAL      = 7'b1101111,
    OP_JALR     = 7'b1100111,
    OP_BRANCH   = 7'b1100011,
    OP_LOAD     = 7'b0000011,
    OP_STORE    = 7'b0100011,
    OP_IMM      = 7'b0010011,
    OP_REG      = 7'b0110011,
    OP_FENCE    = 7'b0001111,
    OP_SYSTEM   = 7'b1110011
  } opcode_e;

  // ── ALU Operations ───────────────────────────────────────────────────
  typedef enum logic [3:0] {
    ALU_ADD   = 4'b0000,
    ALU_SUB   = 4'b1000,
    ALU_SLL   = 4'b0001,
    ALU_SLT   = 4'b0010,
    ALU_SLTU  = 4'b0011,
    ALU_XOR   = 4'b0100,
    ALU_SRL   = 4'b0101,
    ALU_SRA   = 4'b1101,
    ALU_OR    = 4'b0110,
    ALU_AND   = 4'b0111,
    ALU_PASS_B = 4'b1111   // Pass operand B through (for LUI)
  } alu_op_e;

  // ── MUL/DIV Operations (M extension) ─────────────────────────────────
  typedef enum logic [2:0] {
    MD_MUL    = 3'b000,
    MD_MULH   = 3'b001,
    MD_MULHSU = 3'b010,
    MD_MULHU  = 3'b011,
    MD_DIV    = 3'b100,
    MD_DIVU   = 3'b101,
    MD_REM    = 3'b110,
    MD_REMU   = 3'b111
  } md_op_e;

  // ── Writeback source select ──────────────────────────────────────────
  typedef enum logic [2:0] {
    WB_ALU    = 3'd0,
    WB_MEM    = 3'd1,
    WB_PC4    = 3'd2,  // JAL/JALR link
    WB_CSR    = 3'd3,
    WB_MULDIV = 3'd4
  } wb_sel_e;

  // ── Memory access width ──────────────────────────────────────────────
  typedef enum logic [1:0] {
    MEM_BYTE = 2'b00,
    MEM_HALF = 2'b01,
    MEM_WORD = 2'b10
  } mem_width_e;

  // ── Branch type ──────────────────────────────────────────────────────
  typedef enum logic [2:0] {
    BR_NONE = 3'b000,
    BR_EQ   = 3'b001,
    BR_NE   = 3'b010,
    BR_LT   = 3'b011,
    BR_GE   = 3'b100,
    BR_LTU  = 3'b101,
    BR_GEU  = 3'b110
  } branch_type_e;

  // ── Forward select ───────────────────────────────────────────────────
  typedef enum logic [1:0] {
    FWD_NONE  = 2'b00,
    FWD_EX_MEM = 2'b01,
    FWD_MEM_WB = 2'b10
  } fwd_sel_e;

  // ── Pipeline control bundle (ID → EX) ───────────────────────────────
  typedef struct packed {
    // ALU
    alu_op_e      alu_op;
    logic         alu_src;        // 0=rs2, 1=imm
    // M extension
    logic         muldiv_en;
    md_op_e       muldiv_op;
    // Memory
    logic         mem_rd;
    logic         mem_wr;
    mem_width_e   mem_width;
    logic         mem_sign_ext;
    // Writeback
    logic         reg_wr;
    wb_sel_e      wb_sel;
    // Branch/Jump
    branch_type_e branch_type;
    logic         jal;
    logic         jalr;
    // CSR
    logic         csr_en;
    logic [2:0]   csr_op;
    logic [11:0]  csr_addr;
    // System
    logic         ecall;
    logic         ebreak;
    logic         mret;
    logic         fence;
    logic         illegal;
  } ctrl_t;

  // ── CSR Addresses ────────────────────────────────────────────────────
  localparam logic [11:0] CSR_MSTATUS   = 12'h300;
  localparam logic [11:0] CSR_MIE       = 12'h304;
  localparam logic [11:0] CSR_MTVEC     = 12'h305;
  localparam logic [11:0] CSR_MSCRATCH  = 12'h340;
  localparam logic [11:0] CSR_MEPC      = 12'h341;
  localparam logic [11:0] CSR_MCAUSE    = 12'h342;
  localparam logic [11:0] CSR_MTVAL     = 12'h343;
  localparam logic [11:0] CSR_MIP       = 12'h344;
  localparam logic [11:0] CSR_MCYCLE    = 12'hB00;
  localparam logic [11:0] CSR_MINSTRET  = 12'hB02;
  localparam logic [11:0] CSR_MHARTID   = 12'hF14;

  // ── Memory Map ───────────────────────────────────────────────────────
  localparam logic [31:0] RESET_VECTOR  = 32'h0000_0000;
  localparam logic [31:0] IMEM_BASE     = 32'h0000_0000;
  localparam logic [31:0] DMEM_BASE     = 32'h1000_0000;
  localparam logic [31:0] PERIPH_BASE   = 32'h2000_0000;
  localparam logic [31:0] GPIO_BASE     = 32'h2000_0000;
  localparam logic [31:0] UART_BASE     = 32'h2000_0100;
  localparam logic [31:0] TIMER_BASE    = 32'h2000_0200;

  // ── Cache parameters ─────────────────────────────────────────────────
  localparam int ICACHE_SETS     = 64;   // 64 sets
  localparam int ICACHE_WAYS     = 2;    // 2-way set-assoc
  localparam int ICACHE_LINE_W   = 128;  // 128-bit (4-word) cache line
  localparam int DCACHE_SETS     = 64;
  localparam int DCACHE_WAYS     = 2;
  localparam int DCACHE_LINE_W   = 128;

  // ── AXI4-Lite parameters ─────────────────────────────────────────────
  localparam int AXI_ADDR_W = 32;
  localparam int AXI_DATA_W = 32;
  localparam int AXI_STRB_W = AXI_DATA_W / 8;

endpackage
