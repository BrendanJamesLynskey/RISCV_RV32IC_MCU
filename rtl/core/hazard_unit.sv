// ============================================================================
// hazard_unit.sv — Pipeline Hazard Detection & Data Forwarding
// ============================================================================
// Handles:
//  - Data hazards via EX→EX and MEM→EX forwarding
//  - Load-use hazard via 1-cycle pipeline stall
//  - Control hazards via pipeline flush on branch mispredicts
//  - MulDiv stalls
// ============================================================================
import brv32p_pkg::*;

module hazard_unit (
  // ID stage register addresses
  input  logic [4:0]  id_rs1,
  input  logic [4:0]  id_rs2,

  // EX stage info
  input  logic [4:0]  ex_rd,
  input  logic        ex_reg_wr,
  input  logic        ex_mem_rd,     // Load in EX → stall
  input  wb_sel_e     ex_wb_sel,

  // MEM stage info
  input  logic [4:0]  mem_rd,
  input  logic        mem_reg_wr,

  // WB stage info
  input  logic [4:0]  wb_rd,
  input  logic        wb_reg_wr,

  // Control hazards
  input  logic        branch_mispredict,
  input  logic        jump_ex,        // JAL/JALR resolved in EX

  // MulDiv stall
  input  logic        muldiv_busy,

  // Forwarding outputs
  output fwd_sel_e    fwd_rs1,
  output fwd_sel_e    fwd_rs2,

  // Pipeline control
  output logic        stall_if,
  output logic        stall_id,
  output logic        stall_ex,
  output logic        flush_id,
  output logic        flush_ex,
  output logic        flush_mem
);

  // ── Load-use hazard detection ────────────────────────────────────────
  logic load_use_hazard;
  assign load_use_hazard = ex_mem_rd && (ex_rd != 5'd0) &&
                           ((ex_rd == id_rs1) || (ex_rd == id_rs2));

  // ── Data forwarding ──────────────────────────────────────────────────
  always_comb begin
    // RS1 forwarding
    if (ex_reg_wr && (ex_rd != 5'd0) && (ex_rd == id_rs1) && !ex_mem_rd)
      fwd_rs1 = FWD_EX_MEM;
    else if (mem_reg_wr && (mem_rd != 5'd0) && (mem_rd == id_rs1))
      fwd_rs1 = FWD_MEM_WB;
    else
      fwd_rs1 = FWD_NONE;

    // RS2 forwarding
    if (ex_reg_wr && (ex_rd != 5'd0) && (ex_rd == id_rs2) && !ex_mem_rd)
      fwd_rs2 = FWD_EX_MEM;
    else if (mem_reg_wr && (mem_rd != 5'd0) && (mem_rd == id_rs2))
      fwd_rs2 = FWD_MEM_WB;
    else
      fwd_rs2 = FWD_NONE;
  end

  // ── Stall / Flush logic ──────────────────────────────────────────────
  always_comb begin
    stall_if  = 1'b0;
    stall_id  = 1'b0;
    stall_ex  = 1'b0;
    flush_id  = 1'b0;
    flush_ex  = 1'b0;
    flush_mem = 1'b0;

    // MulDiv stall: freeze IF/ID/EX until done
    if (muldiv_busy) begin
      stall_if = 1'b1;
      stall_id = 1'b1;
      stall_ex = 1'b1;
    end
    // Load-use: stall IF and ID, insert bubble in EX
    else if (load_use_hazard) begin
      stall_if = 1'b1;
      stall_id = 1'b1;
      flush_ex = 1'b1;   // Bubble
    end

    // Branch mispredict: flush IF, ID, EX (instructions fetched after branch)
    if (branch_mispredict) begin
      flush_id  = 1'b1;
      flush_ex  = 1'b1;
      // Override stalls — flush takes priority
      stall_if  = 1'b0;
      stall_id  = 1'b0;
    end
  end

endmodule
