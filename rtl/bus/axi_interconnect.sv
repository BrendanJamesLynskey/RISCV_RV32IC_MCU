// ============================================================================
// axi_interconnect.sv — Simple AXI4-Lite Bus Fabric
// ============================================================================
// 2 master ports (I-cache, D-cache) → arbiter → address decoder →
// 2 slave ports (main memory, peripheral bridge).
// D-cache has priority over I-cache.
// Write channel properly holds signals until slave handshake completes.
// Provides m1_wr_done pulse to dcache when write completes.
// ============================================================================

module axi_interconnect (
  input  logic        clk,
  input  logic        rst_n,

  // Master 0: I-Cache (read only)
  input  logic [31:0] m0_araddr,
  input  logic        m0_arvalid,
  output logic        m0_arready,
  output logic [31:0] m0_rdata,
  output logic        m0_rvalid,
  input  logic        m0_rready,

  // Master 1: D-Cache (reads)
  input  logic [31:0] m1_araddr,
  input  logic        m1_arvalid,
  output logic        m1_arready,
  output logic [31:0] m1_rdata,
  output logic        m1_rvalid,
  input  logic        m1_rready,

  // Master 1: D-Cache (writes)
  input  logic [31:0] m1_awaddr,
  input  logic        m1_awvalid,
  output logic        m1_awready,
  input  logic [31:0] m1_wdata,
  input  logic [3:0]  m1_wstrb,
  input  logic        m1_wvalid,
  output logic        m1_wready,
  output logic        m1_bvalid,
  input  logic        m1_bready,

  // Write completion signal to dcache
  output logic        m1_wr_done,

  // Slave 0: Main Memory (0x0000_0000 – 0x1FFF_FFFF)
  output logic [31:0] s0_araddr,
  output logic        s0_arvalid,
  input  logic        s0_arready,
  input  logic [31:0] s0_rdata,
  input  logic        s0_rvalid,
  output logic        s0_rready,
  output logic [31:0] s0_awaddr,
  output logic        s0_awvalid,
  input  logic        s0_awready,
  output logic [31:0] s0_wdata,
  output logic [3:0]  s0_wstrb,
  output logic        s0_wvalid,
  input  logic        s0_wready,
  input  logic        s0_bvalid,
  output logic        s0_bready,

  // Slave 1: Peripherals (0x2000_0000 – 0x2FFF_FFFF)
  output logic [31:0] s1_araddr,
  output logic        s1_arvalid,
  input  logic        s1_arready,
  input  logic [31:0] s1_rdata,
  input  logic        s1_rvalid,
  output logic        s1_rready,
  output logic [31:0] s1_awaddr,
  output logic        s1_awvalid,
  input  logic        s1_awready,
  output logic [31:0] s1_wdata,
  output logic [3:0]  s1_wstrb,
  output logic        s1_wvalid,
  input  logic        s1_wready,
  input  logic        s1_bvalid,
  output logic        s1_bready
);

  // ── Arbiter state ────────────────────────────────────────────────────
  typedef enum logic [2:0] {ARB_IDLE, ARB_M0_RD, ARB_M1_RD, ARB_M1_WR} arb_state_e;
  arb_state_e arb_state;

  // Address decode
  function automatic logic addr_is_periph(input logic [31:0] a);
    return (a[31:28] == 4'h2);
  endfunction

  // ── Registered write request ─────────────────────────────────────────
  // Capture write request on entry to ARB_M1_WR so we can hold signals
  // stable even if the master deasserts them.
  logic [31:0] wr_addr_reg, wr_data_reg;
  logic [3:0]  wr_strb_reg;
  logic        wr_target_periph;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      arb_state        <= ARB_IDLE;
      wr_addr_reg      <= 32'b0;
      wr_data_reg      <= 32'b0;
      wr_strb_reg      <= 4'b0;
      wr_target_periph <= 1'b0;
    end else begin
      case (arb_state)
        ARB_IDLE: begin
          // D-cache write has highest priority
          if (m1_awvalid && m1_wvalid) begin
            arb_state        <= ARB_M1_WR;
            wr_addr_reg      <= m1_awaddr;
            wr_data_reg      <= m1_wdata;
            wr_strb_reg      <= m1_wstrb;
            wr_target_periph <= addr_is_periph(m1_awaddr);
          end else if (m1_arvalid) begin
            arb_state <= ARB_M1_RD;
          end else if (m0_arvalid) begin
            arb_state <= ARB_M0_RD;
          end
        end

        ARB_M0_RD: begin
          logic target;
          target = addr_is_periph(m0_araddr);
          if (target ? (s1_rvalid && m0_rready) : (s0_rvalid && m0_rready))
            arb_state <= ARB_IDLE;
        end

        ARB_M1_RD: begin
          logic target;
          target = addr_is_periph(m1_araddr);
          if (target ? (s1_rvalid && m1_rready) : (s0_rvalid && m1_rready))
            arb_state <= ARB_IDLE;
        end

        ARB_M1_WR: begin
          // Wait for write response from the target slave
          if (wr_target_periph ? (s1_bvalid) : (s0_bvalid))
            arb_state <= ARB_IDLE;
        end

        default: arb_state <= ARB_IDLE;
      endcase
    end
  end

  // ── Write completion pulse ───────────────────────────────────────────
  assign m1_wr_done = (arb_state == ARB_M1_WR) &&
                      (wr_target_periph ? s1_bvalid : s0_bvalid);

  // ── Routing ──────────────────────────────────────────────────────────
  always_comb begin
    // Default: deassert everything
    m0_arready = 1'b0; m0_rdata = 32'b0; m0_rvalid = 1'b0;
    m1_arready = 1'b0; m1_rdata = 32'b0; m1_rvalid = 1'b0;
    m1_awready = 1'b0; m1_wready = 1'b0; m1_bvalid = 1'b0;

    s0_araddr = 32'b0; s0_arvalid = 1'b0; s0_rready = 1'b0;
    s0_awaddr = 32'b0; s0_awvalid = 1'b0; s0_wdata  = 32'b0;
    s0_wstrb  = 4'b0;  s0_wvalid  = 1'b0; s0_bready = 1'b0;

    s1_araddr = 32'b0; s1_arvalid = 1'b0; s1_rready = 1'b0;
    s1_awaddr = 32'b0; s1_awvalid = 1'b0; s1_wdata  = 32'b0;
    s1_wstrb  = 4'b0;  s1_wvalid  = 1'b0; s1_bready = 1'b0;

    case (arb_state)
      ARB_M0_RD: begin
        if (addr_is_periph(m0_araddr)) begin
          s1_araddr  = m0_araddr;  s1_arvalid = m0_arvalid;
          m0_arready = s1_arready; m0_rdata   = s1_rdata;
          m0_rvalid  = s1_rvalid;  s1_rready  = m0_rready;
        end else begin
          s0_araddr  = m0_araddr;  s0_arvalid = m0_arvalid;
          m0_arready = s0_arready; m0_rdata   = s0_rdata;
          m0_rvalid  = s0_rvalid;  s0_rready  = m0_rready;
        end
      end

      ARB_M1_RD: begin
        if (addr_is_periph(m1_araddr)) begin
          s1_araddr  = m1_araddr;  s1_arvalid = m1_arvalid;
          m1_arready = s1_arready; m1_rdata   = s1_rdata;
          m1_rvalid  = s1_rvalid;  s1_rready  = m1_rready;
        end else begin
          s0_araddr  = m1_araddr;  s0_arvalid = m1_arvalid;
          m1_arready = s0_arready; m1_rdata   = s0_rdata;
          m1_rvalid  = s0_rvalid;  s0_rready  = m1_rready;
        end
      end

      ARB_M1_WR: begin
        // Use REGISTERED write data — held stable for the entire handshake
        if (wr_target_periph) begin
          s1_awaddr  = wr_addr_reg;  s1_awvalid = 1'b1;
          s1_wdata   = wr_data_reg;  s1_wstrb   = wr_strb_reg;
          s1_wvalid  = 1'b1;
          m1_awready = s1_awready;   m1_wready  = s1_wready;
          m1_bvalid  = s1_bvalid;    s1_bready  = m1_bready;
        end else begin
          s0_awaddr  = wr_addr_reg;  s0_awvalid = 1'b1;
          s0_wdata   = wr_data_reg;  s0_wstrb   = wr_strb_reg;
          s0_wvalid  = 1'b1;
          m1_awready = s0_awready;   m1_wready  = s0_wready;
          m1_bvalid  = s0_bvalid;    s0_bready  = m1_bready;
        end
      end

      default: ;
    endcase
  end

endmodule
