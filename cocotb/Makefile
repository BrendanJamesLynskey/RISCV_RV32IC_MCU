TOPLEVEL_LANG = verilog
VERILOG_SOURCES = \
    $(PWD)/../rtl/pkg/brv32p_pkg.sv \
    $(PWD)/../rtl/core/alu.sv \
    $(PWD)/../rtl/core/regfile.sv \
    $(PWD)/../rtl/core/compressed_decoder.sv \
    $(PWD)/../rtl/core/decoder.sv \
    $(PWD)/../rtl/core/branch_predictor.sv \
    $(PWD)/../rtl/core/hazard_unit.sv \
    $(PWD)/../rtl/core/muldiv.sv \
    $(PWD)/../rtl/core/csr.sv \
    $(PWD)/../rtl/core/brv32p_core.sv \
    $(PWD)/../rtl/cache/icache.sv \
    $(PWD)/../rtl/cache/dcache.sv \
    $(PWD)/../rtl/bus/axi_interconnect.sv \
    $(PWD)/../rtl/bus/axi_sram.sv \
    $(PWD)/../rtl/bus/axi_periph_bridge.sv \
    $(PWD)/../rtl/periph/gpio.sv \
    $(PWD)/../rtl/periph/uart.sv \
    $(PWD)/../rtl/periph/timer.sv \
    $(PWD)/../rtl/brv32p_soc.sv

TOPLEVEL = brv32p_soc
MODULE = test_brv32p_soc
SIM ?= icarus

ifeq ($(SIM),icarus)
  COMPILE_ARGS += -g2012
endif

include $(shell cocotb-config --makefiles)/Makefile.sim
