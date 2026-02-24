// ============================================================================
// timer.sv — 32-bit Timer/Counter Peripheral
// ============================================================================
// Registers (byte offsets from TIMER_BASE):
//   0x00  CTRL     — [0] Enable, [1] Auto-reload
//   0x04  PRESCALER— Clock prescaler (tick every prescaler+1 clocks)
//   0x08  COMPARE  — Compare/reload value
//   0x0C  COUNT    — Current counter value (R/W)
//   0x10  STATUS   — [0] Compare match flag (W1C)
// ============================================================================

module timer (
  input  logic        clk,
  input  logic        rst_n,

  // Bus interface
  input  logic [7:0]  addr,
  input  logic        wr_en,
  input  logic        rd_en,
  input  logic [31:0] wdata,
  output logic [31:0] rdata,

  output logic        irq
);

  logic        enable;
  logic        auto_reload;
  logic [31:0] prescaler;
  logic [31:0] compare;
  logic [31:0] count;
  logic [31:0] pre_cnt;
  logic        match_flag;

  // ── Tick generation ───────────────────────────────────────────────
  wire tick = enable && (pre_cnt == prescaler);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      enable      <= 1'b0;
      auto_reload <= 1'b0;
      prescaler   <= 32'b0;
      compare     <= 32'hFFFF_FFFF;
      count       <= 32'b0;
      pre_cnt     <= 32'b0;
      match_flag  <= 1'b0;
    end else begin
      // Prescaler
      if (enable) begin
        if (pre_cnt >= prescaler)
          pre_cnt <= 32'b0;
        else
          pre_cnt <= pre_cnt + 1'b1;
      end

      // Counter
      if (tick) begin
        if (count >= compare) begin
          match_flag <= 1'b1;
          count <= auto_reload ? 32'b0 : count;
        end else begin
          count <= count + 1'b1;
        end
      end

      // Register writes
      if (wr_en) begin
        case (addr[4:2])
          3'd0: {auto_reload, enable} <= wdata[1:0];
          3'd1: prescaler <= wdata;
          3'd2: compare   <= wdata;
          3'd3: count     <= wdata;
          3'd4: match_flag <= match_flag & ~wdata[0]; // W1C
          default: ;
        endcase
      end
    end
  end

  // ── Register Read ─────────────────────────────────────────────────
  always_comb begin
    rdata = 32'b0;
    if (rd_en) begin
      case (addr[4:2])
        3'd0: rdata = {30'b0, auto_reload, enable};
        3'd1: rdata = prescaler;
        3'd2: rdata = compare;
        3'd3: rdata = count;
        3'd4: rdata = {31'b0, match_flag};
        default: rdata = 32'b0;
      endcase
    end
  end

  assign irq = match_flag;

endmodule
