// ============================================================================
// axi_periph_bridge.sv — AXI4-Lite to Simple Bus Peripheral Bridge
// ============================================================================
module axi_periph_bridge (
  input  logic        clk,
  input  logic        rst_n,
  // AXI read
  input  logic [31:0] araddr,  input  logic arvalid, output logic arready,
  output logic [31:0] rdata,   output logic rvalid,  input  logic rready,
  // AXI write
  input  logic [31:0] awaddr,  input  logic awvalid, output logic awready,
  input  logic [31:0] wdata,   input  logic [3:0] wstrb,
  input  logic        wvalid,  output logic wready,
  output logic        bvalid,  input  logic bready,
  // GPIO
  output logic [7:0]  gpio_addr, output logic gpio_rd, output logic gpio_wr,
  output logic [31:0] gpio_wdata, input logic [31:0] gpio_rdata,
  // UART
  output logic [7:0]  uart_addr, output logic uart_rd, output logic uart_wr,
  output logic [31:0] uart_wdata, input logic [31:0] uart_rdata,
  // Timer
  output logic [7:0]  timer_addr, output logic timer_rd, output logic timer_wr,
  output logic [31:0] timer_wdata, input logic [31:0] timer_rdata
);

  function automatic logic [1:0] psel(input logic [31:0] a);
    case (a[11:8]) 4'h0: return 2'd0; 4'h1: return 2'd1; 4'h2: return 2'd2; default: return 2'd0; endcase
  endfunction

  // Read FSM
  typedef enum logic [1:0] {RI, RA, RR} rfsm_e;
  rfsm_e rfsm;
  logic [31:0] rdr, rar;
  assign arready = (rfsm == RI);
  assign rdata = rdr;
  assign rvalid = (rfsm == RR);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin rfsm <= RI; rdr <= 0; rar <= 0; end
    else case (rfsm)
      RI: if (arvalid) begin rar <= araddr; rfsm <= RA; end
      RA: begin
        case (psel(rar)) 2'd0: rdr<=gpio_rdata; 2'd1: rdr<=uart_rdata; 2'd2: rdr<=timer_rdata; default: rdr<=0; endcase
        rfsm <= RR;
      end
      RR: if (rready) rfsm <= RI;
      default: rfsm <= RI;
    endcase
  end

  always_comb begin
    gpio_rd=0; uart_rd=0; timer_rd=0;
    gpio_addr=rar[7:0]; uart_addr=rar[7:0]; timer_addr=rar[7:0];
    if (rfsm==RA) case(psel(rar)) 2'd0:gpio_rd=1; 2'd1:uart_rd=1; 2'd2:timer_rd=1; default:; endcase
  end

  // Write FSM
  typedef enum logic [1:0] {WI, WA, WR} wfsm_e;
  wfsm_e wfsm;
  logic [31:0] war, wdr;
  assign awready = (wfsm==WI) && awvalid && wvalid;
  assign wready  = (wfsm==WI) && awvalid && wvalid;
  assign bvalid  = (wfsm==WR);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin wfsm<=WI; war<=0; wdr<=0; end
    else case (wfsm)
      WI: if (awvalid&&wvalid) begin war<=awaddr; wdr<=wdata; wfsm<=WA; end
      WA: wfsm <= WR;
      WR: if (bready) wfsm <= WI;
      default: wfsm <= WI;
    endcase
  end

  always_comb begin
    gpio_wr=0; uart_wr=0; timer_wr=0;
    gpio_wdata=wdr; uart_wdata=wdr; timer_wdata=wdr;
    if (wfsm==WA) begin
      gpio_addr=war[7:0]; uart_addr=war[7:0]; timer_addr=war[7:0];
      case(psel(war)) 2'd0:gpio_wr=1; 2'd1:uart_wr=1; 2'd2:timer_wr=1; default:; endcase
    end
  end

endmodule
