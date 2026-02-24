// ============================================================================
// axi_sram.sv — AXI4-Lite SRAM Slave (Unified backing memory)
// ============================================================================
// Single-port SRAM with 1-cycle read latency, word-addressable.
// Handles both instruction and data regions.
// Initialised from hex file for instruction region.
// ============================================================================

module axi_sram #(
  parameter DEPTH     = 8192,     // Words (32 KB)
  parameter INIT_FILE = "firmware.hex"
)(
  input  logic        clk,
  input  logic        rst_n,

  // AXI4-Lite slave: Read
  input  logic [31:0] araddr,
  input  logic        arvalid,
  output logic        arready,
  output logic [31:0] rdata,
  output logic        rvalid,
  input  logic        rready,

  // AXI4-Lite slave: Write
  input  logic [31:0] awaddr,
  input  logic        awvalid,
  output logic        awready,
  input  logic [31:0] wdata,
  input  logic [3:0]  wstrb,
  input  logic        wvalid,
  output logic        wready,
  output logic        bvalid,
  input  logic        bready
);

  localparam AW = $clog2(DEPTH);

  logic [31:0] mem [0:DEPTH-1];

  // Initialise
  initial begin
    for (int i = 0; i < DEPTH; i++)
      mem[i] = 32'h0000_0013; // NOP
    if (INIT_FILE != "")
      $readmemh(INIT_FILE, mem);
  end

  // ── Read channel ─────────────────────────────────────────────────────
  typedef enum logic [1:0] {RD_IDLE, RD_RESP} rd_state_e;
  rd_state_e rd_state;
  logic [31:0] rd_data_reg;

  assign arready = (rd_state == RD_IDLE);
  assign rdata   = rd_data_reg;
  assign rvalid  = (rd_state == RD_RESP);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rd_state    <= RD_IDLE;
      rd_data_reg <= 32'b0;
    end else begin
      case (rd_state)
        RD_IDLE: begin
          if (arvalid) begin
            rd_data_reg <= mem[araddr[AW+1:2]];
            rd_state    <= RD_RESP;
          end
        end
        RD_RESP: begin
          if (rready)
            rd_state <= RD_IDLE;
        end
      endcase
    end
  end

  // ── Write channel ────────────────────────────────────────────────────
  typedef enum logic [1:0] {WR_IDLE, WR_RESP} wr_state_e;
  wr_state_e wr_state;

  assign awready = (wr_state == WR_IDLE) && awvalid && wvalid;
  assign wready  = (wr_state == WR_IDLE) && awvalid && wvalid;
  assign bvalid  = (wr_state == WR_RESP);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wr_state <= WR_IDLE;
    end else begin
      case (wr_state)
        WR_IDLE: begin
          if (awvalid && wvalid) begin
            for (int i = 0; i < 4; i++)
              if (wstrb[i])
                mem[awaddr[AW+1:2]][i*8 +: 8] <= wdata[i*8 +: 8];
            wr_state <= WR_RESP;
          end
        end
        WR_RESP: begin
          if (bready)
            wr_state <= WR_IDLE;
        end
      endcase
    end
  end

endmodule
