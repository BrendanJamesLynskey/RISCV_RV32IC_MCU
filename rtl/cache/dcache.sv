// ============================================================================
// dcache.sv — 2-Way Set-Associative Data Cache (Write-Through)
// ============================================================================
// 64 sets × 2 ways × 16 bytes/line = 2 KB
// Write-through, write-allocate policy. Sub-word access handled.
// ============================================================================
import brv32p_pkg::*;

module dcache #(
  parameter SETS       = 64,
  parameter WAYS       = 2,
  parameter LINE_BYTES = 16
)(
  input  logic        clk,
  input  logic        rst_n,

  // CPU interface
  input  logic [31:0] addr,
  input  logic        rd_en,
  input  logic        wr_en,
  input  mem_width_e  width,
  input  logic        sign_ext,
  input  logic [31:0] wdata,
  output logic [31:0] rdata,
  output logic        ready,

  // Memory interface
  output logic [31:0] mem_addr,
  output logic        mem_rd,
  output logic        mem_wr,
  output logic [31:0] mem_wdata,
  output logic [3:0]  mem_wstrb,
  input  logic [31:0] mem_rdata,
  input  logic        mem_valid
);

  localparam OFFSET_W = $clog2(LINE_BYTES);
  localparam SET_W    = $clog2(SETS);
  localparam TAG_W    = 32 - SET_W - OFFSET_W;
  localparam WORDS    = LINE_BYTES / 4;

  // ── Storage ──────────────────────────────────────────────────────────
  logic [TAG_W-1:0] tag_mem  [0:SETS-1][0:WAYS-1];
  logic             valid_mem[0:SETS-1][0:WAYS-1];
  logic [31:0]      data_mem [0:SETS-1][0:WAYS-1][0:WORDS-1];
  logic             lru      [0:SETS-1];

  // ── Address decomposition ────────────────────────────────────────────
  logic [TAG_W-1:0]    tag;
  logic [SET_W-1:0]    set_idx;
  logic [OFFSET_W-3:0] word_sel;
  logic [1:0]          byte_off;

  assign tag      = addr[31:SET_W+OFFSET_W];
  assign set_idx  = addr[SET_W+OFFSET_W-1:OFFSET_W];
  assign word_sel = addr[OFFSET_W-1:2];
  assign byte_off = addr[1:0];

  // ── Hit detection ────────────────────────────────────────────────────
  logic hit0, hit1, hit;
  logic hit_way;
  assign hit0    = valid_mem[set_idx][0] && (tag_mem[set_idx][0] == tag);
  assign hit1    = valid_mem[set_idx][1] && (tag_mem[set_idx][1] == tag);
  assign hit     = hit0 || hit1;
  assign hit_way = hit1; // 0 if way0 hit, 1 if way1 hit

  // ── Sub-word read logic ──────────────────────────────────────────────
  logic [31:0] raw_word;
  assign raw_word = hit0 ? data_mem[set_idx][0][word_sel] :
                           data_mem[set_idx][1][word_sel];

  always_comb begin
    rdata = 32'b0;
    case (width)
      MEM_BYTE: begin
        logic [7:0] b;
        case (byte_off)
          2'd0: b = raw_word[7:0];
          2'd1: b = raw_word[15:8];
          2'd2: b = raw_word[23:16];
          2'd3: b = raw_word[31:24];
        endcase
        rdata = sign_ext ? {{24{b[7]}}, b} : {24'b0, b};
      end
      MEM_HALF: begin
        logic [15:0] h;
        h = byte_off[1] ? raw_word[31:16] : raw_word[15:0];
        rdata = sign_ext ? {{16{h[15]}}, h} : {16'b0, h};
      end
      MEM_WORD: rdata = raw_word;
      default:  rdata = raw_word;
    endcase
  end

  // ── Write strobe generation ──────────────────────────────────────────
  logic [3:0] wstrb;
  always_comb begin
    case (width)
      MEM_BYTE: wstrb = 4'b0001 << byte_off;
      MEM_HALF: wstrb = byte_off[1] ? 4'b1100 : 4'b0011;
      MEM_WORD: wstrb = 4'b1111;
      default:  wstrb = 4'b1111;
    endcase
  end

  // Align write data
  logic [31:0] wdata_aligned;
  always_comb begin
    case (width)
      MEM_BYTE: wdata_aligned = {4{wdata[7:0]}};
      MEM_HALF: wdata_aligned = {2{wdata[15:0]}};
      MEM_WORD: wdata_aligned = wdata;
      default:  wdata_aligned = wdata;
    endcase
  end

  // ── FSM ──────────────────────────────────────────────────────────────
  typedef enum logic [2:0] {S_IDLE, S_FILL, S_FILL_DONE, S_WRITE_THROUGH} state_e;
  state_e state;
  logic [1:0] fill_cnt;
  logic       fill_way;
  logic [31:0] pending_addr;
  logic [31:0] pending_wdata;
  logic [3:0]  pending_wstrb;
  logic        pending_is_write;

  always_comb begin
    ready     = 1'b0;
    mem_rd    = 1'b0;
    mem_wr    = 1'b0;
    mem_addr  = 32'b0;
    mem_wdata = 32'b0;
    mem_wstrb = 4'b0;

    case (state)
      S_IDLE: begin
        if ((rd_en || wr_en) && hit) begin
          ready = 1'b1;
          // Write-through: also send write to memory
          if (wr_en) begin
            mem_wr    = 1'b1;
            mem_addr  = addr;
            mem_wdata = wdata_aligned;
            mem_wstrb = wstrb;
            // For simplicity, write-through doesn't stall on hit
            // (fire-and-forget — real designs would need a write buffer)
            ready = 1'b1;
          end
        end else if ((rd_en || wr_en) && !hit) begin
          ready = 1'b0; // Miss
        end else begin
          ready = 1'b1;
        end
      end
      S_FILL: begin
        mem_rd   = 1'b1;
        mem_addr = {pending_addr[31:OFFSET_W], fill_cnt, 2'b00};
      end
      S_FILL_DONE: begin
        ready = 1'b1;
        if (pending_is_write) begin
          mem_wr    = 1'b1;
          mem_addr  = pending_addr;
          mem_wdata = pending_wdata;
          mem_wstrb = pending_wstrb;
        end
      end
      default: ;
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state    <= S_IDLE;
      fill_cnt <= 2'b0;
      fill_way <= 1'b0;
      pending_is_write <= 1'b0;
      for (int s = 0; s < SETS; s++) begin
        for (int w = 0; w < WAYS; w++)
          valid_mem[s][w] <= 1'b0;
        lru[s] <= 1'b0;
      end
    end else begin
      case (state)
        S_IDLE: begin
          if ((rd_en || wr_en) && hit) begin
            lru[set_idx] <= hit_way ? 1'b0 : 1'b1;
            // Update cache on write hit
            if (wr_en) begin
              logic [31:0] old_word;
              old_word = data_mem[set_idx][hit_way][word_sel];
              for (int i = 0; i < 4; i++)
                if (wstrb[i])
                  data_mem[set_idx][hit_way][word_sel][i*8 +: 8] <= wdata_aligned[i*8 +: 8];
            end
          end else if ((rd_en || wr_en) && !hit) begin
            fill_way         <= lru[set_idx] ? 1'b1 : 1'b0;
            fill_cnt         <= 2'b0;
            pending_addr     <= addr;
            pending_wdata    <= wdata_aligned;
            pending_wstrb    <= wstrb;
            pending_is_write <= wr_en;
            state            <= S_FILL;
          end
        end

        S_FILL: begin
          if (mem_valid) begin
            data_mem[set_idx][fill_way][fill_cnt] <= mem_rdata;
            if (fill_cnt == 2'(WORDS - 1)) begin
              tag_mem[set_idx][fill_way]   <= tag;
              valid_mem[set_idx][fill_way] <= 1'b1;
              lru[set_idx]                 <= fill_way ? 1'b0 : 1'b1;
              state                        <= S_FILL_DONE;
            end else begin
              fill_cnt <= fill_cnt + 1'b1;
            end
          end
        end

        S_FILL_DONE: begin
          // Apply pending write to freshly-filled line
          if (pending_is_write) begin
            for (int i = 0; i < 4; i++)
              if (pending_wstrb[i])
                data_mem[set_idx][fill_way][word_sel][i*8 +: 8] <= pending_wdata[i*8 +: 8];
          end
          state <= S_IDLE;
        end

        default: state <= S_IDLE;
      endcase
    end
  end

endmodule
