// ============================================================================
// axi4lite_if.sv — AXI4-Lite Interface Definition
// ============================================================================
// Reusable interface for connecting masters and slaves on an AXI4-Lite bus.
// ============================================================================

interface axi4lite_if #(
  parameter ADDR_W = 32,
  parameter DATA_W = 32
);

  localparam STRB_W = DATA_W / 8;

  // Write address channel
  logic [ADDR_W-1:0] awaddr;
  logic               awvalid;
  logic               awready;

  // Write data channel
  logic [DATA_W-1:0] wdata;
  logic [STRB_W-1:0] wstrb;
  logic               wvalid;
  logic               wready;

  // Write response channel
  logic [1:0]         bresp;
  logic               bvalid;
  logic               bready;

  // Read address channel
  logic [ADDR_W-1:0] araddr;
  logic               arvalid;
  logic               arready;

  // Read data channel
  logic [DATA_W-1:0] rdata;
  logic [1:0]         rresp;
  logic               rvalid;
  logic               rready;

  modport master (
    output awaddr, awvalid, input  awready,
    output wdata,  wstrb,   wvalid, input  wready,
    input  bresp,  bvalid,  output bready,
    output araddr, arvalid, input  arready,
    input  rdata,  rresp,   rvalid, output rready
  );

  modport slave (
    input  awaddr, awvalid, output awready,
    input  wdata,  wstrb,   wvalid, output wready,
    output bresp,  bvalid,  input  bready,
    input  araddr, arvalid, output arready,
    output rdata,  rresp,   rvalid, input  rready
  );

endinterface
