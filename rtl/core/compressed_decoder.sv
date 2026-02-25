// ============================================================================
// compressed_decoder.sv — RV32C Compressed Instruction Expander
// ============================================================================
// Expands 16-bit compressed instructions into their 32-bit equivalents.
// If the instruction is already 32-bit (bits [1:0] == 2'b11), it passes
// through unchanged. Returns 'is_compressed' to adjust PC increment.
// ============================================================================

module compressed_decoder (
  input  logic [31:0] instr_in,     // Raw fetched data (may be 16 or 32 bit)
  output logic [31:0] instr_out,    // Expanded 32-bit instruction
  output logic        is_compressed, // 1 if input was 16-bit
  output logic        illegal_c     // Illegal compressed encoding
);

  logic [15:0] ci;  // Compressed instruction
  assign ci = instr_in[15:0];
  assign is_compressed = (ci[1:0] != 2'b11);

  // Compressed register mapping: cr' = cr + 8 (registers x8–x15)
  function automatic logic [4:0] cr(input logic [2:0] r);
    return {2'b01, r};
  endfunction

  always_comb begin
    instr_out = instr_in;  // Default: pass through 32-bit
    illegal_c = 1'b0;

    if (is_compressed) begin
      instr_out = 32'h0000_0013;  // NOP default (illegal → NOP to be safe)
      illegal_c = 1'b0;

      case (ci[1:0])
        // ── Quadrant 0 ──────────────────────────────────────────────
        2'b00: begin
          case (ci[15:13])
            3'b000: begin // C.ADDI4SPN → addi rd', x2, nzuimm
              if (ci[12:5] == 8'b0) begin
                illegal_c = 1'b1;
              end else begin
                logic [9:0] nzuimm;
                nzuimm = {ci[10:7], ci[12:11], ci[5], ci[6], 2'b00};
                instr_out = {2'b0, nzuimm, 5'd2, 3'b000, cr(ci[4:2]), 7'b0010011};
              end
            end
            3'b010: begin // C.LW → lw rd', offset(rs1')
              logic [6:0] off;
              off = {ci[5], ci[12:10], ci[6], 2'b00};
              instr_out = {5'b0, off, cr(ci[9:7]), 3'b010, cr(ci[4:2]), 7'b0000011};
            end
            3'b110: begin // C.SW → sw rs2', offset(rs1')
              logic [6:0] off;
              off = {ci[5], ci[12:10], ci[6], 2'b00};
              instr_out = {5'b0, off[6:5], cr(ci[4:2]), cr(ci[9:7]), 3'b010, off[4:0], 7'b0100011};
            end
            default: illegal_c = 1'b1;
          endcase
        end

        // ── Quadrant 1 ──────────────────────────────────────────────
        2'b01: begin
          case (ci[15:13])
            3'b000: begin // C.ADDI / C.NOP → addi rd, rd, nzimm
              logic [5:0] imm;
              imm = {ci[12], ci[6:2]};
              instr_out = {{6{imm[5]}}, imm, ci[11:7], 3'b000, ci[11:7], 7'b0010011};
            end
            3'b001: begin // C.JAL → jal x1, offset (RV32 only)
              logic [20:0] jimm;
              // Sign-extend ci[12] through bits [20:12], then the offset fields
              jimm = {{10{ci[12]}}, ci[8], ci[10:9], ci[6], ci[7], ci[2], ci[11], ci[5:3], 1'b0};
              instr_out = {jimm[20], jimm[10:1], jimm[11], jimm[19:12], 5'd1, 7'b1101111};
            end
            3'b010: begin // C.LI → addi rd, x0, imm
              logic [5:0] imm;
              imm = {ci[12], ci[6:2]};
              instr_out = {{6{imm[5]}}, imm, 5'd0, 3'b000, ci[11:7], 7'b0010011};
            end
            3'b011: begin
              if (ci[11:7] == 5'd2) begin // C.ADDI16SP → addi x2, x2, nzimm
                logic [9:0] nzimm;
                nzimm = {ci[12], ci[4:3], ci[5], ci[2], ci[6], 4'b0000};
                instr_out = {{2{nzimm[9]}}, nzimm, 5'd2, 3'b000, 5'd2, 7'b0010011};
              end else begin // C.LUI → lui rd, nzuimm
                logic [17:0] nzuimm;
                nzuimm = {ci[12], ci[6:2], 12'b0};
                instr_out = {{14{ci[12]}}, ci[12], ci[6:2], ci[11:7], 7'b0110111};
              end
            end
            3'b100: begin // ALU operations on compressed registers
              case (ci[11:10])
                2'b00: begin // C.SRLI → srli rd', rd', shamt
                  instr_out = {7'b0000000, ci[6:2], cr(ci[9:7]), 3'b101, cr(ci[9:7]), 7'b0010011};
                end
                2'b01: begin // C.SRAI → srai rd', rd', shamt
                  instr_out = {7'b0100000, ci[6:2], cr(ci[9:7]), 3'b101, cr(ci[9:7]), 7'b0010011};
                end
                2'b10: begin // C.ANDI → andi rd', rd', imm
                  logic [5:0] imm;
                  imm = {ci[12], ci[6:2]};
                  instr_out = {{6{imm[5]}}, imm, cr(ci[9:7]), 3'b111, cr(ci[9:7]), 7'b0010011};
                end
                2'b11: begin
                  case ({ci[12], ci[6:5]})
                    3'b000: instr_out = {7'b0100000, cr(ci[4:2]), cr(ci[9:7]), 3'b000, cr(ci[9:7]), 7'b0110011}; // C.SUB
                    3'b001: instr_out = {7'b0000000, cr(ci[4:2]), cr(ci[9:7]), 3'b100, cr(ci[9:7]), 7'b0110011}; // C.XOR
                    3'b010: instr_out = {7'b0000000, cr(ci[4:2]), cr(ci[9:7]), 3'b110, cr(ci[9:7]), 7'b0110011}; // C.OR
                    3'b011: instr_out = {7'b0000000, cr(ci[4:2]), cr(ci[9:7]), 3'b111, cr(ci[9:7]), 7'b0110011}; // C.AND
                    default: illegal_c = 1'b1;
                  endcase
                end
              endcase
            end
            3'b101: begin // C.J → jal x0, offset
              logic [20:0] jimm;
              // Sign-extend ci[12] through bits [20:12], then the offset fields
              jimm = {{10{ci[12]}}, ci[8], ci[10:9], ci[6], ci[7], ci[2], ci[11], ci[5:3], 1'b0};
              instr_out = {jimm[20], jimm[10:1], jimm[11], jimm[19:12], 5'd0, 7'b1101111};
            end
            3'b110: begin // C.BEQZ → beq rs1', x0, offset
              logic [8:0] boff;
              boff = {ci[12], ci[6:5], ci[2], ci[11:10], ci[4:3], 1'b0};
              // B-type: {imm[12], imm[10:5], rs2, rs1, funct3, imm[4:1], imm[11], opcode}
              // Sign-extend boff[8] into imm[12:9]
              instr_out = {boff[8], {3{boff[8]}}, boff[7:5], 5'd0, cr(ci[9:7]), 3'b000, boff[4:1], boff[8], 7'b1100011};
            end
            3'b111: begin // C.BNEZ → bne rs1', x0, offset
              logic [8:0] boff;
              boff = {ci[12], ci[6:5], ci[2], ci[11:10], ci[4:3], 1'b0};
              // B-type: {imm[12], imm[10:5], rs2, rs1, funct3, imm[4:1], imm[11], opcode}
              // Sign-extend boff[8] into imm[12:9]
              instr_out = {boff[8], {3{boff[8]}}, boff[7:5], 5'd0, cr(ci[9:7]), 3'b001, boff[4:1], boff[8], 7'b1100011};
            end
            default: illegal_c = 1'b1;
          endcase
        end

        // ── Quadrant 2 ──────────────────────────────────────────────
        2'b10: begin
          case (ci[15:13])
            3'b000: begin // C.SLLI → slli rd, rd, shamt
              instr_out = {7'b0000000, ci[6:2], ci[11:7], 3'b001, ci[11:7], 7'b0010011};
            end
            3'b010: begin // C.LWSP → lw rd, offset(x2)
              logic [7:0] off;
              off = {ci[3:2], ci[12], ci[6:4], 2'b00};
              instr_out = {4'b0, off, 5'd2, 3'b010, ci[11:7], 7'b0000011};
            end
            3'b100: begin
              if (ci[12] == 1'b0) begin
                if (ci[6:2] == 5'b0) begin // C.JR → jalr x0, rs1, 0
                  instr_out = {12'b0, ci[11:7], 3'b000, 5'd0, 7'b1100111};
                end else begin // C.MV → add rd, x0, rs2
                  instr_out = {7'b0, ci[6:2], 5'd0, 3'b000, ci[11:7], 7'b0110011};
                end
              end else begin
                if (ci[6:2] == 5'b0) begin
                  if (ci[11:7] == 5'b0) begin // C.EBREAK → ebreak
                    instr_out = 32'h0010_0073;
                  end else begin // C.JALR → jalr x1, rs1, 0
                    instr_out = {12'b0, ci[11:7], 3'b000, 5'd1, 7'b1100111};
                  end
                end else begin // C.ADD → add rd, rd, rs2
                  instr_out = {7'b0, ci[6:2], ci[11:7], 3'b000, ci[11:7], 7'b0110011};
                end
              end
            end
            3'b110: begin // C.SWSP → sw rs2, offset(x2)
              logic [7:0] off;
              off = {ci[8:7], ci[12:9], 2'b00};
              instr_out = {4'b0, off[7:5], ci[6:2], 5'd2, 3'b010, off[4:0], 7'b0100011};
            end
            default: illegal_c = 1'b1;
          endcase
        end

        default: begin
          // bits [1:0] == 2'b11 → 32-bit, handled by default assignment
        end
      endcase
    end
  end

endmodule
