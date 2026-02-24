// ============================================================================
// alu.sv — 32-bit ALU for BRV32P
// ============================================================================
import brv32p_pkg::*;

module alu (
  input  logic [31:0] a,
  input  logic [31:0] b,
  input  alu_op_e     op,
  output logic [31:0] result,
  output logic        zero
);

  always_comb begin
    case (op)
      ALU_ADD:    result = a + b;
      ALU_SUB:    result = a - b;
      ALU_SLL:    result = a << b[4:0];
      ALU_SLT:    result = {31'b0, $signed(a) < $signed(b)};
      ALU_SLTU:   result = {31'b0, a < b};
      ALU_XOR:    result = a ^ b;
      ALU_SRL:    result = a >> b[4:0];
      ALU_SRA:    result = $unsigned($signed(a) >>> b[4:0]);
      ALU_OR:     result = a | b;
      ALU_AND:    result = a & b;
      ALU_PASS_B: result = b;
      default:    result = 32'b0;
    endcase
  end

  assign zero = (result == 32'b0);

endmodule
