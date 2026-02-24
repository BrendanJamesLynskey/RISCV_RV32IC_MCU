// ============================================================================
// csr.sv — Control and Status Register Unit for BRV32P
// ============================================================================
import brv32p_pkg::*;

module csr (
  input  logic        clk,
  input  logic        rst_n,

  // CSR access (from WB stage)
  input  logic        csr_en,
  input  logic [11:0] csr_addr,
  input  logic [2:0]  csr_op,
  input  logic [31:0] csr_wdata,
  output logic [31:0] csr_rdata,

  // Trap interface
  input  logic        trap_enter,
  input  logic [31:0] trap_cause,
  input  logic [31:0] trap_val,
  input  logic [31:0] trap_pc,
  output logic [31:0] mtvec_out,
  output logic [31:0] mepc_out,

  // MRET
  input  logic        mret,

  // Interrupts
  input  logic        ext_irq,
  input  logic        timer_irq,
  input  logic        instr_retired,
  output logic        irq_pending
);

  logic [31:0] mstatus, mie, mtvec, mscratch, mepc, mcause, mtval;
  logic [31:0] mip;
  logic [63:0] mcycle, minstret;

  assign mtvec_out = mtvec;
  assign mepc_out  = mepc;

  // Interrupt pending
  always_comb begin
    mip = 32'b0;
    mip[11] = ext_irq;
    mip[7]  = timer_irq;
  end
  assign irq_pending = mstatus[3] & |(mip & mie);

  // CSR Read
  always_comb begin
    csr_rdata = 32'b0;
    case (csr_addr)
      CSR_MSTATUS:  csr_rdata = mstatus;
      CSR_MIE:      csr_rdata = mie;
      CSR_MTVEC:    csr_rdata = mtvec;
      CSR_MSCRATCH: csr_rdata = mscratch;
      CSR_MEPC:     csr_rdata = mepc;
      CSR_MCAUSE:   csr_rdata = mcause;
      CSR_MTVAL:    csr_rdata = mtval;
      CSR_MIP:      csr_rdata = mip;
      CSR_MCYCLE:   csr_rdata = mcycle[31:0];
      CSR_MINSTRET: csr_rdata = minstret[31:0];
      CSR_MHARTID:  csr_rdata = 32'd0;
      default:      csr_rdata = 32'b0;
    endcase
  end

  // CSR Write / Trap
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mstatus  <= 32'b0;
      mie      <= 32'b0;
      mtvec    <= 32'b0;
      mscratch <= 32'b0;
      mepc     <= 32'b0;
      mcause   <= 32'b0;
      mtval    <= 32'b0;
      mcycle   <= 64'b0;
      minstret <= 64'b0;
    end else begin
      mcycle <= mcycle + 1'b1;
      if (instr_retired)
        minstret <= minstret + 1'b1;

      if (trap_enter) begin
        mepc       <= trap_pc;
        mcause     <= trap_cause;
        mtval      <= trap_val;
        mstatus[7] <= mstatus[3];
        mstatus[3] <= 1'b0;
      end else if (mret) begin
        mstatus[3] <= mstatus[7];
        mstatus[7] <= 1'b1;
      end else if (csr_en) begin
        logic [31:0] nv;
        case (csr_op[1:0])
          2'b01: nv = csr_wdata;
          2'b10: nv = csr_rdata | csr_wdata;
          2'b11: nv = csr_rdata & ~csr_wdata;
          default: nv = csr_rdata;
        endcase
        case (csr_addr)
          CSR_MSTATUS:  mstatus  <= nv & 32'h88;
          CSR_MIE:      mie      <= nv;
          CSR_MTVEC:    mtvec    <= {nv[31:2], 2'b00};
          CSR_MSCRATCH: mscratch <= nv;
          CSR_MEPC:     mepc     <= {nv[31:2], 2'b00};
          CSR_MCAUSE:   mcause   <= nv;
          CSR_MTVAL:    mtval    <= nv;
          default: ;
        endcase
      end
    end
  end

endmodule
