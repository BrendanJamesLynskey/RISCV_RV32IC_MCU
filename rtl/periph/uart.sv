// ============================================================================
// uart.sv — Minimal UART Peripheral (TX + RX, 8N1)
// ============================================================================
// Registers (byte offsets from UART_BASE):
//   0x00  TX_DATA  — Write byte to transmit (WO)
//   0x04  RX_DATA  — Read received byte (RO)
//   0x08  STATUS   — [0] TX busy, [1] RX data valid, [2] RX overrun
//   0x0C  CTRL     — [15:0] Clock divider (baud = clk / (div + 1))
// ============================================================================

module uart (
  input  logic        clk,
  input  logic        rst_n,

  // Bus interface
  input  logic [7:0]  addr,
  input  logic        wr_en,
  input  logic        rd_en,
  input  logic [31:0] wdata,
  output logic [31:0] rdata,

  // Serial pins
  output logic        uart_tx,
  input  logic        uart_rx,
  output logic        irq
);

  // ── Configuration ──────────────────────────────────────────────────
  logic [15:0] clk_div;

  // ── TX state machine ──────────────────────────────────────────────
  typedef enum logic [1:0] {TX_IDLE, TX_START, TX_DATA, TX_STOP} tx_state_e;
  tx_state_e tx_state;
  logic [7:0]  tx_shift;
  logic [2:0]  tx_bit_cnt;
  logic [15:0] tx_clk_cnt;
  logic        tx_busy;

  // ── RX state machine ──────────────────────────────────────────────
  typedef enum logic [1:0] {RX_IDLE, RX_START, RX_DATA, RX_STOP} rx_state_e;
  rx_state_e rx_state;
  logic [7:0]  rx_shift;
  logic [7:0]  rx_data;
  logic [2:0]  rx_bit_cnt;
  logic [15:0] rx_clk_cnt;
  logic        rx_valid;
  logic        rx_overrun;

  // Double-sync RX input
  logic rx_sync, rx_meta;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_meta <= 1'b1;
      rx_sync <= 1'b1;
    end else begin
      rx_meta <= uart_rx;
      rx_sync <= rx_meta;
    end
  end

  // ── TX Logic ──────────────────────────────────────────────────────
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_state   <= TX_IDLE;
      uart_tx    <= 1'b1;
      tx_busy    <= 1'b0;
      tx_shift   <= 8'b0;
      tx_bit_cnt <= 3'b0;
      tx_clk_cnt <= 16'b0;
    end else begin
      case (tx_state)
        TX_IDLE: begin
          uart_tx <= 1'b1;
          if (wr_en && addr[3:2] == 2'd0) begin
            tx_shift   <= wdata[7:0];
            tx_busy    <= 1'b1;
            tx_state   <= TX_START;
            tx_clk_cnt <= clk_div;
          end
        end

        TX_START: begin
          uart_tx <= 1'b0; // Start bit
          if (tx_clk_cnt == 16'b0) begin
            tx_clk_cnt <= clk_div;
            tx_bit_cnt <= 3'd0;
            tx_state   <= TX_DATA;
          end else begin
            tx_clk_cnt <= tx_clk_cnt - 1'b1;
          end
        end

        TX_DATA: begin
          uart_tx <= tx_shift[0];
          if (tx_clk_cnt == 16'b0) begin
            tx_clk_cnt <= clk_div;
            tx_shift   <= {1'b0, tx_shift[7:1]};
            if (tx_bit_cnt == 3'd7) begin
              tx_state <= TX_STOP;
            end else begin
              tx_bit_cnt <= tx_bit_cnt + 1'b1;
            end
          end else begin
            tx_clk_cnt <= tx_clk_cnt - 1'b1;
          end
        end

        TX_STOP: begin
          uart_tx <= 1'b1; // Stop bit
          if (tx_clk_cnt == 16'b0) begin
            tx_busy  <= 1'b0;
            tx_state <= TX_IDLE;
          end else begin
            tx_clk_cnt <= tx_clk_cnt - 1'b1;
          end
        end
      endcase
    end
  end

  // ── RX Logic ──────────────────────────────────────────────────────
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_state   <= RX_IDLE;
      rx_valid   <= 1'b0;
      rx_overrun <= 1'b0;
      rx_data    <= 8'b0;
      rx_shift   <= 8'b0;
      rx_bit_cnt <= 3'b0;
      rx_clk_cnt <= 16'b0;
    end else begin
      // Clear valid on read
      if (rd_en && addr[3:2] == 2'd1)
        rx_valid <= 1'b0;

      case (rx_state)
        RX_IDLE: begin
          if (~rx_sync) begin // Start bit detected
            rx_clk_cnt <= {1'b0, clk_div[15:1]}; // Sample at mid-bit
            rx_state   <= RX_START;
          end
        end

        RX_START: begin
          if (rx_clk_cnt == 16'b0) begin
            if (~rx_sync) begin // Confirm start bit
              rx_clk_cnt <= clk_div;
              rx_bit_cnt <= 3'd0;
              rx_state   <= RX_DATA;
            end else begin
              rx_state <= RX_IDLE; // False start
            end
          end else begin
            rx_clk_cnt <= rx_clk_cnt - 1'b1;
          end
        end

        RX_DATA: begin
          if (rx_clk_cnt == 16'b0) begin
            rx_clk_cnt <= clk_div;
            rx_shift   <= {rx_sync, rx_shift[7:1]};
            if (rx_bit_cnt == 3'd7) begin
              rx_state <= RX_STOP;
            end else begin
              rx_bit_cnt <= rx_bit_cnt + 1'b1;
            end
          end else begin
            rx_clk_cnt <= rx_clk_cnt - 1'b1;
          end
        end

        RX_STOP: begin
          if (rx_clk_cnt == 16'b0) begin
            if (rx_sync) begin // Valid stop bit
              if (rx_valid)
                rx_overrun <= 1'b1;
              rx_data  <= rx_shift;
              rx_valid <= 1'b1;
            end
            rx_state <= RX_IDLE;
          end else begin
            rx_clk_cnt <= rx_clk_cnt - 1'b1;
          end
        end
      endcase
    end
  end

  // ── Register Read ─────────────────────────────────────────────────
  always_comb begin
    rdata = 32'b0;
    if (rd_en) begin
      case (addr[3:2])
        2'd0: rdata = 32'b0;                           // TX_DATA (WO)
        2'd1: rdata = {24'b0, rx_data};                 // RX_DATA
        2'd2: rdata = {29'b0, rx_overrun, rx_valid, tx_busy}; // STATUS
        2'd3: rdata = {16'b0, clk_div};                 // CTRL
      endcase
    end
  end

  // ── Register Write (ctrl) ─────────────────────────────────────────
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      clk_div <= 16'd867; // Default: 115200 @ 100MHz
    end else if (wr_en && addr[3:2] == 2'd3) begin
      clk_div <= wdata[15:0];
    end
  end

  assign irq = rx_valid;

endmodule
