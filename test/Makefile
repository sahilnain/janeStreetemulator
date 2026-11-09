# Makefile
# See https://docs.cocotb.org/en/stable/quickstart.html for more info
#
# Top-level tests for tt_um_sahil_protoemu (test.py through tb.v).
#   make             RTL simulation
#   make GATES=yes   gate-level simulation (after the GDS flow)
# Unit tests for each module live in unit/ (run: cd unit && pytest).

# Protocol support (docs/spec/blocks.md Section 13). UART is always built:
#   make PROTOCOLS=uart          UART only
#   make PROTOCOLS=uart,i2c      UART and I2C
# The RTL gets -DSUPPORT_<P>=0 for each protocol left out, the Python model reads PE_PROTOCOLS, and
# a test that needs a protocol which is not built is skipped (test/common/pe_build.py).
PROTOCOLS ?= uart,spi,i2c,usb
comma := ,
PROTOCOL_LIST := $(subst $(comma), ,$(PROTOCOLS))
ifneq ($(filter-out uart spi i2c usb,$(PROTOCOL_LIST)),)
$(error PROTOCOLS: unknown $(filter-out uart spi i2c usb,$(PROTOCOL_LIST)); known: uart spi i2c usb)
endif
export PE_PROTOCOLS := $(PROTOCOLS)
PROTOCOL_DEFINES := $(if $(filter spi,$(PROTOCOL_LIST)),,-DSUPPORT_SPI=0) \
                    $(if $(filter i2c,$(PROTOCOL_LIST)),,-DSUPPORT_I2C=0) \
                    $(if $(filter usb,$(PROTOCOL_LIST)),,-DSUPPORT_USB=0)
# one build directory per protocol set, so a change of PROTOCOLS never reuses a stale build
BUILD_TAG := $(subst $(comma),_,$(PROTOCOLS))

# defaults
SIM ?= icarus
FST ?= -fst # Use more efficient FST format
TOPLEVEL_LANG ?= verilog
SRC_DIR = $(PWD)/../src
PROJECT_SOURCES = tt_um_sahil_protoemu.v \
                  host/pe_host_if.v host/pe_regs.v \
                  mem/pe_imem.v mem/pe_fifo.v mem/pe_dmem.v \
                  io/pe_sync.v io/pe_pins.v \
                  core/pe_ctrl.v core/pe_decode.v core/pe_vote.v \
                  unit/pe_txu.v unit/pe_rxu.v unit/pe_det.v unit/pe_crc.v

ifneq ($(GATES),yes)

# RTL simulation:
SIM_BUILD				?= sim_build/rtl_$(BUILD_TAG)
VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))
COMPILE_ARGS    += $(PROTOCOL_DEFINES)

else

# Gate level simulation:
SIM_BUILD				= sim_build/gl
COMPILE_ARGS    += -DGL_TEST
COMPILE_ARGS    += -DFUNCTIONAL
COMPILE_ARGS    += -DSIM
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13g2/libs.ref/sg13g2_io/verilog/sg13g2_io.v
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13g2/libs.ref/sg13g2_stdcell/verilog/sg13g2_stdcell.v

# this gets copied in by the GDS action workflow
VERILOG_SOURCES += $(PWD)/gate_level_netlist.v

endif

# Allow sharing configuration between design and testbench via `include`:
COMPILE_ARGS 		+= -I$(SRC_DIR)

# Include the testbench sources (TB=tb_dual: two emulators, test_dual.py):
TB ?= tb
VERILOG_SOURCES += $(PWD)/$(TB).v
TOPLEVEL = $(TB)

# List test modules to run, separated by commas and without the .py suffix:
COCOTB_TEST_MODULES ?= test,test_uart,test_spi,test_i2c,test_modes,test_usb,test_fabric,test_gaps,test_lockstep,test_cross

# include cocotb's make rules to take care of the simulator setup
include $(shell cocotb-config --makefiles)/Makefile.sim
