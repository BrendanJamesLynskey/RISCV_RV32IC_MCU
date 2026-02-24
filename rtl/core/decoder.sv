// ============================================================================
// decoder.sv — RV32IMC Instruction Decoder for BRV32P
// ============================================================================
// Decodes a 32-bit instruction (already expanded from C) into the ctrl_t
// control bundle plus register addresses and immediate value.
// ============================================================================
import brv32p_pkg::*;

module decoder (
  input  logic [31:0] instr,
  output logic [4:0]  rs1_addr,
  output logic [4:0]  rs2_addr,
  output logic [4:0]  rd_addr,
  output logic [31:0] imm,
  output ctrl_t       ctrl
);

  opcode_e opcode;
  logic [2:0] funct3;
  logic [6:0] funct7;

  assign opcode   = opcode_e'(instr[6:0]);
  assign funct3   = instr[14:12];
  assign funct7   = instr[31:25];
  assign rs1_addr = instr[19:15];
  assign rs2_addr = instr[24:20];
  assign rd_addr  = instr[11:7];

  // ── Immediate generation ─────────────────────────────────────────────
  always_comb begin
    case (opcode)
      OP_LOAD, OP_JALR, OP_IMM, OP_SYSTEM:
        imm = {{20{instr[31]}}, instr[31:20]};
      OP_STORE:
        imm = {{20{instr[31]}}, instr[31:25], instr[11:7]};
      OP_BRANCH:
        imm = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
      OP_LUI, OP_AUIPC:
        imm = {instr[31:12], 12'b0};
      OP_JAL:
        imm = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
      default:
        imm = 32'b0;
    endcase
  end

  // ── Control signal generation ────────────────────────────────────────
  always_comb begin
    // Defaults — everything off
    ctrl = '0;

    case (opcode)
      OP_LUI: begin
        ctrl.reg_wr  = 1'b1;
        ctrl.alu_op  = ALU_PASS_B;
        ctrl.alu_src = 1'b1;
        ctrl.wb_sel  = WB_ALU;
      end

      OP_AUIPC: begin
        ctrl.reg_wr  = 1'b1;
        ctrl.alu_op  = ALU_ADD;
        ctrl.alu_src = 1'b1;  // Will use PC as operand A externally
        ctrl.wb_sel  = WB_ALU;
      end

      OP_JAL: begin
        ctrl.jal     = 1'b1;
        ctrl.reg_wr  = 1'b1;
        ctrl.wb_sel  = WB_PC4;
      end

      OP_JALR: begin
        ctrl.jalr    = 1'b1;
        ctrl.reg_wr  = 1'b1;
        ctrl.alu_src = 1'b1;
        ctrl.alu_op  = ALU_ADD;
        ctrl.wb_sel  = WB_PC4;
      end

      OP_BRANCH: begin
        case (funct3)
          3'b000: ctrl.branch_type = BR_EQ;
          3'b001: ctrl.branch_type = BR_NE;
          3'b100: ctrl.branch_type = BR_LT;
          3'b101: ctrl.branch_type = BR_GE;
          3'b110: ctrl.branch_type = BR_LTU;
          3'b111: ctrl.branch_type = BR_GEU;
          default: ctrl.illegal = 1'b1;
        endcase
      end

      OP_LOAD: begin
        ctrl.mem_rd      = 1'b1;
        ctrl.reg_wr      = 1'b1;
        ctrl.alu_src     = 1'b1;
        ctrl.alu_op      = ALU_ADD;
        ctrl.wb_sel      = WB_MEM;
        ctrl.mem_width   = mem_width_e'(funct3[1:0]);
        ctrl.mem_sign_ext = ~funct3[2];
      end

      OP_STORE: begin
        ctrl.mem_wr    = 1'b1;
        ctrl.alu_src   = 1'b1;
        ctrl.alu_op    = ALU_ADD;
        ctrl.mem_width = mem_width_e'(funct3[1:0]);
      end

      OP_IMM: begin
        ctrl.reg_wr  = 1'b1;
        ctrl.alu_src = 1'b1;
        ctrl.wb_sel  = WB_ALU;
        case (funct3)
          3'b000: ctrl.alu_op = ALU_ADD;
          3'b001: ctrl.alu_op = ALU_SLL;
          3'b010: ctrl.alu_op = ALU_SLT;
          3'b011: ctrl.alu_op = ALU_SLTU;
          3'b100: ctrl.alu_op = ALU_XOR;
          3'b101: ctrl.alu_op = funct7[5] ? ALU_SRA : ALU_SRL;
          3'b110: ctrl.alu_op = ALU_OR;
          3'b111: ctrl.alu_op = ALU_AND;
        endcase
      end

      OP_REG: begin
        ctrl.reg_wr = 1'b1;
        ctrl.wb_sel = WB_ALU;
        if (funct7 == 7'b0000001) begin
          // M extension
          ctrl.muldiv_en = 1'b1;
          ctrl.muldiv_op = md_op_e'(funct3);
          ctrl.wb_sel    = WB_MULDIV;
        end else begin
          case (funct3)
            3'b000: ctrl.alu_op = funct7[5] ? ALU_SUB : ALU_ADD;
            3'b001: ctrl.alu_op = ALU_SLL;
            3'b010: ctrl.alu_op = ALU_SLT;
            3'b011: ctrl.alu_op = ALU_SLTU;
            3'b100: ctrl.alu_op = ALU_XOR;
            3'b101: ctrl.alu_op = funct7[5] ? ALU_SRA : ALU_SRL;
            3'b110: ctrl.alu_op = ALU_OR;
            3'b111: ctrl.alu_op = ALU_AND;
          endcase
        end
      end

      OP_SYSTEM: begin
        if (funct3 == 3'b000) begin
          case (instr[31:20])
            12'h000: ctrl.ecall  = 1'b1;
            12'h001: ctrl.ebreak = 1'b1;
            12'h302: ctrl.mret   = 1'b1;
            default: ctrl.illegal = 1'b1;
          endcase
        end else begin
          ctrl.csr_en   = 1'b1;
          ctrl.csr_op   = funct3;
          ctrl.csr_addr = instr[31:20];
          ctrl.reg_wr   = 1'b1;
          ctrl.wb_sel   = WB_CSR;
        end
      end

      OP_FENCE: begin
        ctrl.fence = 1'b1;
      end

      default: begin
        ctrl.illegal = 1'b1;
      end
    endcase
  end

endmodule
