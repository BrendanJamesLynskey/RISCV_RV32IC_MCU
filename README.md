# BRV32P — 5-Stage Pipelined RV32IMC RISC-V Microcontroller

A high-performance pipelined RISC-V SoC with caches and AXI4-Lite bus.

## Features
- 5-stage pipeline (IF/ID/EX/MEM/WB) with full data forwarding
- RV32IMC: Base Integer + Multiply/Divide + Compressed instructions
- 2-bit branch predictor with Branch Target Buffer
- 2-way set-associative I-cache and D-cache (2 KB each)
- AXI4-Lite bus interconnect with priority arbiter
- 32 KB unified backing SRAM
- GPIO, UART (8N1), Timer peripherals

## Directory Structure
```
brv32p/
├── rtl/
│   ├── pkg/brv32p_pkg.sv           # Shared types and parameters
│   ├── core/
│   │   ├── brv32p_core.sv          # 5-stage pipeline top
│   │   ├── decoder.sv              # RV32IMC decoder
│   │   ├── compressed_decoder.sv   # RV32C expander
│   │   ├── alu.sv                  # Arithmetic logic unit
│   │   ├── regfile.sv              # 32x32 register file
│   │   ├── muldiv.sv               # M-extension multiply/divide
│   │   ├── branch_predictor.sv     # 2-bit BHT + BTB
│   │   ├── hazard_unit.sv          # Forwarding + stall logic
│   │   └── csr.sv                  # Machine-mode CSRs
│   ├── cache/
│   │   ├── icache.sv               # 2-way I-cache
│   │   └── dcache.sv               # 2-way D-cache (write-through)
│   ├── bus/
│   │   ├── axi4lite_if.sv          # AXI4-Lite interface definition
│   │   ├── axi_interconnect.sv     # 2M→2S bus arbiter
│   │   ├── axi_sram.sv             # AXI SRAM slave
│   │   └── axi_periph_bridge.sv    # AXI → peripheral bridge
│   ├── periph/
│   │   ├── gpio.sv                 # GPIO with interrupts
│   │   ├── uart.sv                 # UART TX/RX
│   │   └── timer.sv                # Timer/counter
│   └── brv32p_soc.sv               # SoC top-level
├── tb/tb_brv32p_soc.sv             # SystemVerilog testbench
├── cocotb/
│   ├── test_brv32p_soc.py          # CocoTB test suite
│   └── Makefile
├── firmware/
│   ├── firmware.hex
│   └── gen_firmware.py
└── doc/BRV32P_Design_Report.md
```

## Running Tests

### SystemVerilog (Icarus Verilog)
```bash
cd tb
iverilog -g2012 -o sim \
  ../rtl/pkg/brv32p_pkg.sv ../rtl/core/*.sv ../rtl/cache/*.sv \
  ../rtl/bus/axi_interconnect.sv ../rtl/bus/axi_sram.sv \
  ../rtl/bus/axi_periph_bridge.sv ../rtl/periph/*.sv \
  ../rtl/brv32p_soc.sv tb_brv32p_soc.sv
cp ../firmware/firmware.hex .
vvp sim +VCD
```

### CocoTB
```bash
cd cocotb
cp ../firmware/firmware.hex .
make
```
