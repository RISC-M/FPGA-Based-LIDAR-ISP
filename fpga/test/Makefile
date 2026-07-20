# Makefile for Cocotb + Verilator Simulation

SIM ?= verilator
TOPLEVEL_LANG ?= verilog

# Paths to the Verilog source files
VERILOG_SOURCES += $(PWD)/../verilog/accelerator.sv
VERILOG_SOURCES += $(PWD)/../verilog/stage_t.sv
VERILOG_SOURCES += $(PWD)/../verilog/stage_f.sv
VERILOG_SOURCES += $(PWD)/../verilog/stage_rmw.sv
VERILOG_SOURCES += $(PWD)/../verilog/trig_lut.sv
VERILOG_SOURCES += $(PWD)/../verilog/double_buffer.sv
VERILOG_SOURCES += $(PWD)/../verilog/mem.sv

# Top-level module name in the Verilog code
TOPLEVEL = accelerator

# Python test module name
MODULE = test_accelerator_holistic

# Verilator specific flags
EXTRA_ARGS += -I$(PWD)/../verilog -y $(PWD)/../verilog --trace --trace-structs -Wno-fatal

include $(shell cocotb-config --makefiles)/Makefile.sim

# Custom target to ensure the LUT ROM file is copied to the simulation build folder
# before running the test, otherwise $readmemh inside Verilator will fail.
$(COCOTB_RESULTS_FILE): sim_build/azimuth_sincos.mem

sim_build/azimuth_sincos.mem: $(PWD)/../verilog/azimuth_sincos.mem
	mkdir -p sim_build
	cp $< $@
