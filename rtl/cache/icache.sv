// ============================================================================
// icache.sv — 2-Way Set-Associative Instruction Cache
// ============================================================================
// 64 sets × 2 ways × 16 bytes/line = 2 KB
// LRU replacement policy (1 bit per set for 2-way).
// Read-only (no write path). Miss triggers AXI4-Lite burst fill.
// ============================================================================
import brv32p_pkg::*;

module icache #(
  parameter SETS      = 64,
  parameter WAYS      = 2,
  parameter LINE_BYTES = 16   // 4 words per line
)(
  input  logic        clk,
  input  logic        rst_n,

  // CPU interface
  input  logic [31:0] addr,
  input  logic        rd_en,
  output logic [31:0] rdata,
  output logic        ready,

  // Memory interface (simplified: word-at-a-time fill)
  output logic [31:0] mem_addr,
  output logic        mem_rd,
  input  logic [31:0] mem_rdata,
  input  logic        mem_valid
);

  localparam OFFSET_W = $clog2(LINE_BYTES);   // 4 bits (byte within line)
  localparam SET_W    = $clog2(SETS);          // 6 bits
  localparam TAG_W    = 32 - SET_W - OFFSET_W; // 22 bits
  localparam WORDS    = LINE_BYTES / 4;        // 4 words per line

  // ── Storage ──────────────────────────────────────────────────────────
  logic [TAG_W-1:0]    tag_mem  [0:SETS-1][0:WAYS-1];
  logic                valid_mem[0:SETS-1][0:WAYS-1];
  logic [31:0]         data_mem [0:SETS-1][0:WAYS-1][0:WORDS-1];
  logic                lru      [0:SETS-1]; // 0 = way0 is LRU

  // ── Address decomposition ────────────────────────────────────────────
  logic [TAG_W-1:0]    tag;
  logic [SET_W-1:0]    set;
  logic [OFFSET_W-3:0] word_sel; // Word index within line (bits [3:2])

  assign tag      = addr[31:SET_W+OFFSET_W];
  assign set      = addr[SET_W+OFFSET_W-1:OFFSET_W];
  assign word_sel = addr[OFFSET_W-1:2];

  // ── Hit detection ────────────────────────────────────────────────────
  logic hit_way0, hit_way1, hit;
  assign hit_way0 = valid_mem[set][0] && (tag_mem[set][0] == tag);
  assign hit_way1 = valid_mem[set][1] && (tag_mem[set][1] == tag);
  assign hit      = hit_way0 || hit_way1;

  // ── FSM ──────────────────────────────────────────────────────────────
  typedef enum logic [1:0] {S_IDLE, S_FILL, S_DONE} state_e;
  state_e state;
  logic [1:0] fill_word_cnt;
  logic       fill_way;   // Which way to fill into
  logic [31:0] fill_base_addr;

  always_comb begin
    ready  = 1'b0;
    rdata  = 32'b0;
    mem_rd = 1'b0;
    mem_addr = 32'b0;

    case (state)
      S_IDLE: begin
        if (rd_en && hit) begin
          ready = 1'b1;
          rdata = hit_way0 ? data_mem[set][0][word_sel] :
                             data_mem[set][1][word_sel];
        end else if (rd_en && !hit) begin
          // Miss — start fill
          ready = 1'b0;
        end else begin
          ready = 1'b1; // No request
        end
      end
      S_FILL: begin
        mem_rd   = 1'b1;
        mem_addr = {fill_base_addr[31:OFFSET_W], fill_word_cnt, 2'b00};
        ready    = 1'b0;
      end
      S_DONE: begin
        ready = 1'b1;
        rdata = data_mem[set][fill_way][word_sel];
      end
      default: begin
        ready = 1'b0;
      end
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state         <= S_IDLE;
      fill_word_cnt <= 2'b0;
      fill_way      <= 1'b0;
      fill_base_addr <= 32'b0;
      for (int s = 0; s < SETS; s++) begin
        for (int w = 0; w < WAYS; w++) begin
          valid_mem[s][w] <= 1'b0;
          tag_mem[s][w]   <= '0;
        end
        lru[s] <= 1'b0;
      end
    end else begin
      case (state)
        S_IDLE: begin
          if (rd_en && hit) begin
            // Update LRU
            lru[set] <= hit_way0 ? 1'b1 : 1'b0; // Point away from hit
          end else if (rd_en && !hit) begin
            // Start fill
            fill_way       <= lru[set] ? 1'b1 : 1'b0; // Evict LRU way
            fill_word_cnt  <= 2'b0;
            fill_base_addr <= addr;
            state          <= S_FILL;
          end
        end

        S_FILL: begin
          if (mem_valid) begin
            data_mem[set][fill_way][fill_word_cnt] <= mem_rdata;
            if (fill_word_cnt == 2'(WORDS - 1)) begin
              tag_mem[set][fill_way]   <= tag;
              valid_mem[set][fill_way] <= 1'b1;
              lru[set]                 <= fill_way ? 1'b0 : 1'b1;
              state                    <= S_DONE;
            end else begin
              fill_word_cnt <= fill_word_cnt + 1'b1;
            end
          end
        end

        S_DONE: begin
          state <= S_IDLE;
        end

        default: state <= S_IDLE;
      endcase
    end
  end

endmodule
