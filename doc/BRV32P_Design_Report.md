# BRV32P — Pipelined RV32IMC RISC-V Microcontroller — Design Report

**Version 1.0 — February 2026**

---

## 1. Introduction

The BRV32P is a high-performance evolution of the BRV32 single-cycle core. It implements the RV32IMC instruction set (Base Integer + Multiply/Divide + Compressed) in a classic 5-stage pipeline with full data forwarding, branch prediction, and a cache hierarchy connected via an AXI4-Lite bus fabric.

| Parameter | BRV32 (original) | BRV32P (this design) |
|---|---|---|
| ISA | RV32I | RV32IMC |
| Pipeline | Single-cycle | 5-stage (IF/ID/EX/MEM/WB) |
| Branch handling | Combinational | 2-bit BHT + BTB |
| Data forwarding | Write-first regfile | Full EX-EX, MEM-EX |
| Multiply | Not supported | Single-cycle (DSP) |
| Divide | Not supported | Iterative (33 cycles) |
| Compressed | Not supported | 16-bit expansion in IF |
| I-Cache | Tightly coupled ROM | 2 KB, 2-way set-assoc |
| D-Cache | Tightly coupled SRAM | 2 KB, 2-way write-through |
| Bus | Direct wiring | AXI4-Lite interconnect |
| Backing memory | Separate I/D mem | Unified 32 KB SRAM |
| Est. throughput | 1x (baseline) | ~5-10x at same clock |

## 2. Pipeline Architecture

### 2.1 Five Stages

**IF (Instruction Fetch):** Reads from I-cache. Compressed instructions expanded. Branch predictor provides speculative next-PC.

**ID (Instruction Decode):** Instruction decoded into ctrl_t control bundle. Register file read.

**EX (Execute):** ALU, branch resolution, multiply/divide. Forwarding MUXes select bypassed data.

**MEM (Memory Access):** D-cache read/write. CSR operations. Trap detection.

**WB (Write-Back):** Result MUX selects ALU/memory/PC+4/CSR/muldiv and writes register file.

### 2.2 Hazard Resolution

**Data hazards:** EX-EX and MEM-EX forwarding. Load-use: 1-cycle bubble.

**Control hazards:** Branch mispredict costs 1 cycle. Predictor reduces penalty on loops.

**Structural hazards:** Divider stalls pipeline for 33 cycles.

### 2.3 Branch Predictor

256-entry BHT (2-bit saturating counters, indexed by PC[9:2]). 64-entry BTB stores last target. On BTB hit + taken prediction, IF fetches speculatively. Mispredict flushes pipeline.

## 3. RV32C Compressed Instructions

The compressed decoder expands 16-bit instructions in the IF stage. Supports Quadrant 0 (C.ADDI4SPN, C.LW, C.SW), Quadrant 1 (C.ADDI, C.JAL, C.LI, C.LUI, C.ADDI16SP, C.SRLI, C.SRAI, C.ANDI, C.SUB/XOR/OR/AND, C.J, C.BEQZ, C.BNEZ), and Quadrant 2 (C.SLLI, C.LWSP, C.JR, C.MV, C.JALR, C.ADD, C.EBREAK, C.SWSP).

## 4. M Extension

Single-cycle multiply (MUL, MULH, MULHSU, MULHU) via 32x32->64 combinational multiplier. 33-cycle iterative restoring divider (DIV, DIVU, REM, REMU). Division by zero returns -1 / dividend per spec.

## 5. Cache Hierarchy

**I-Cache:** 2-way, 64 sets, 16B lines = 2 KB. LRU replacement. 4-word fill on miss.

**D-Cache:** 2-way, 64 sets, 16B lines = 2 KB. Write-through, write-allocate. Sub-word access.

## 6. AXI4-Lite Bus

Priority arbiter: D-cache write > D-cache read > I-cache read. Routes to unified SRAM (0x0-0x1FFF_FFFF) or peripheral bridge (0x2000_0000+).

## 7. Verification

SV testbench (14 checks) and CocoTB (12 tests) covering ALU, forwarding, load/store through cache, branches with prediction, JAL, loops, GPIO, CSR, I-cache fills, BHT training, and pipeline throughput measurement (IPC).
