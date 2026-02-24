// ============================================================================
// regfile.sv — 32x32 Register File for BRV32P
// ============================================================================
// Two read ports (combinational), one write port (synchronous).
// x0 hardwired to zero. No internal forwarding (handled by hazard unit).
// ============================================================================

module regfile (
  input  logic        clk,
  input  logic        rst_n,

  // Read ports (combinational)
  input  logic [4:0]  rs1_addr,
  output logic [31:0] rs1_data,
  input  logic [4:0]  rs2_addr,
  output logic [31:0] rs2_data,

  // Write port (synchronous, rising edge)
  input  logic        wr_en,
  input  logic [4:0]  rd_addr,
  input  logic [31:0] rd_data
);

  logic [31:0] regs [1:31];

  // Read port A
  assign rs1_data = (rs1_addr == 5'd0) ? 32'd0 : regs[rs1_addr];

  // Read port B
  assign rs2_data = (rs2_addr == 5'd0) ? 32'd0 : regs[rs2_addr];

  // Write
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 1; i < 32; i++)
        regs[i] <= 32'd0;
    end else if (wr_en && rd_addr != 5'd0) begin
      regs[rd_addr] <= rd_data;
    end
  end

endmodule
