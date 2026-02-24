// ============================================================================
// muldiv.sv — Multiply / Divide Unit (RV32M Extension)
// ============================================================================
// Single-cycle multiply (synthesises to DSP on FPGA), iterative divide (33 cy).
// Stalls the pipeline via 'busy' signal during multi-cycle division.
// ============================================================================
import brv32p_pkg::*;

module muldiv (
  input  logic        clk,
  input  logic        rst_n,

  input  logic        start,
  input  md_op_e      op,
  input  logic [31:0] a,          // rs1
  input  logic [31:0] b,          // rs2
  output logic [31:0] result,
  output logic        busy,
  output logic        valid       // Result ready this cycle
);

  // ── Multiply (single-cycle) ───────────────────────────────────────────
  logic signed [63:0] mul_ss;
  logic signed [63:0] mul_su;
  logic        [63:0] mul_uu;

  assign mul_ss = $signed(a) * $signed(b);
  assign mul_su = $signed(a) * $signed({1'b0, b});
  assign mul_uu = $unsigned(a) * $unsigned(b);

  // ── Divide (iterative, 33 cycles) ────────────────────────────────────
  logic        div_active;
  logic [5:0]  div_count;
  logic        div_signed;
  logic        div_rem;
  logic        negate_result;
  logic        negate_remainder;
  logic [31:0] dividend;
  logic [31:0] divisor_reg;
  logic [63:0] div_acc;    // {remainder, quotient} shift register

  logic [31:0] div_result;
  logic        div_done;

  assign div_done = div_active && (div_count == 6'd33);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      div_active       <= 1'b0;
      div_count        <= 6'd0;
      div_acc          <= 64'd0;
      divisor_reg      <= 32'd0;
      div_signed       <= 1'b0;
      div_rem          <= 1'b0;
      negate_result    <= 1'b0;
      negate_remainder <= 1'b0;
    end else if (start && (op inside {MD_DIV, MD_DIVU, MD_REM, MD_REMU})) begin
      div_active <= 1'b1;
      div_count  <= 6'd0;

      div_signed <= (op == MD_DIV) || (op == MD_REM);
      div_rem    <= (op == MD_REM) || (op == MD_REMU);

      // Handle signs for signed division
      if ((op == MD_DIV) || (op == MD_REM)) begin
        logic [31:0] abs_a, abs_b;
        abs_a = a[31] ? (~a + 1) : a;
        abs_b = b[31] ? (~b + 1) : b;
        div_acc     <= {32'd0, abs_a};
        divisor_reg <= abs_b;
        negate_result    <= a[31] ^ b[31];
        negate_remainder <= a[31];
      end else begin
        div_acc     <= {32'd0, a};
        divisor_reg <= b;
        negate_result    <= 1'b0;
        negate_remainder <= 1'b0;
      end
    end else if (div_active) begin
      if (div_count < 6'd33) begin
        // Restoring division step
        logic [32:0] trial;
        logic [63:0] shifted;
        shifted = {div_acc[62:0], 1'b0};
        trial = shifted[63:31] - {1'b0, divisor_reg};
        if (!trial[32]) begin
          div_acc <= {trial[31:0], shifted[30:0], 1'b1};
        end else begin
          div_acc <= shifted;
        end
        div_count <= div_count + 1'b1;
      end else begin
        div_active <= 1'b0;
      end
    end
  end

  // Division result select
  always_comb begin
    if (div_rem) begin
      div_result = negate_remainder ? (~div_acc[63:32] + 1) : div_acc[63:32];
    end else begin
      div_result = negate_result ? (~div_acc[31:0] + 1) : div_acc[31:0];
    end
    // Division by zero
    if (divisor_reg == 32'd0) begin
      if (div_rem)
        div_result = dividend;
      else
        div_result = 32'hFFFF_FFFF;
    end
  end

  // ── Output MUX ────────────────────────────────────────────────────────
  logic is_mul;
  assign is_mul = (op inside {MD_MUL, MD_MULH, MD_MULHSU, MD_MULHU});

  always_comb begin
    valid  = 1'b0;
    busy   = 1'b0;
    result = 32'b0;

    if (start && is_mul) begin
      valid = 1'b1;
      case (op)
        MD_MUL:    result = mul_ss[31:0];
        MD_MULH:   result = mul_ss[63:32];
        MD_MULHSU: result = mul_su[63:32];
        MD_MULHU:  result = mul_uu[63:32];
        default:   result = 32'b0;
      endcase
    end else if (div_active) begin
      busy = 1'b1;
      if (div_done) begin
        valid  = 1'b1;
        busy   = 1'b0;
        result = div_result;
      end
    end
  end

  // Store original dividend for div-by-zero remainder
  always_ff @(posedge clk) begin
    if (start)
      dividend <= a;
  end

endmodule
