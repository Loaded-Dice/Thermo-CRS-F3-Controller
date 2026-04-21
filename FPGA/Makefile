SHELL := cmd
.SHELLFLAGS := /C

TOP ?= rs485_Gateway
SRC ?= rs485_Gateway.v
PCF ?= rs485_Gateway.pcf
DEVICE ?= hx8k
PACKAGE ?= cb132
BUILD_DIR ?= build
OSSCAD_BIN ?= C:/apio/packages/oss-cad-suite/bin
YOSYS ?= $(OSSCAD_BIN)/yosys.exe
NEXTPNR ?= $(OSSCAD_BIN)/nextpnr-ice40.exe
ICEPACK ?= $(OSSCAD_BIN)/icepack.exe
ICEPROG ?= $(OSSCAD_BIN)/iceprog.exe
ICEFUNPROG ?= $(USERPROFILE)/.platformio/penv/Scripts/icefunprog.exe
NETLISTSVG_NODE ?= node
NETLISTSVG_SCRIPT ?= $(APPDATA)\npm\node_modules\netlistsvg\bin\netlistsvg.js
SERIAL_PORT ?=
SHOW_PREFIX ?= $(BUILD_DIR)\$(TOP)
SHOW_STUB ?= show_pll_stub.v

JSON := $(BUILD_DIR)\$(TOP).json
ASC := $(BUILD_DIR)\$(TOP).asc
BIN := $(BUILD_DIR)\$(TOP).bin
SHOW_JSON := $(BUILD_DIR)/$(TOP)_schematic.json
SHOW_SVG := $(BUILD_DIR)/$(TOP)_schematic.svg

.PHONY: all synth show pnr bin prog clean

all: bin

$(BUILD_DIR):
	if not exist $(BUILD_DIR) mkdir $(BUILD_DIR)

synth: $(JSON)

$(JSON): $(SRC) | $(BUILD_DIR)
	"$(YOSYS)" -p "synth_ice40 -top $(TOP) -json $(JSON)" $(SRC)

show: $(SHOW_SVG)
	start "" "$(SHOW_SVG)"

$(SHOW_JSON): $(SRC) $(SHOW_STUB) | $(BUILD_DIR)
	"$(YOSYS)" -p "read_verilog $(SHOW_STUB); read_verilog $(SRC); prep -top $(TOP); write_json $(SHOW_JSON)"

$(SHOW_SVG): $(SHOW_JSON)
	"$(NETLISTSVG_NODE)" --stack_size=65500 "$(NETLISTSVG_SCRIPT)" "$(SHOW_JSON)" -o "$(SHOW_SVG)"

pnr: $(ASC)

$(ASC): $(JSON) $(PCF)
	"$(NEXTPNR)" --$(DEVICE) --package $(PACKAGE) --pcf $(PCF) --json $(JSON) --asc $(ASC)

bin: $(BIN)

$(BIN): $(ASC)
	"$(ICEPACK)" $(ASC) $(BIN)

prog: $(BIN)
ifeq ($(strip $(SERIAL_PORT)),)
	@echo ERROR: SERIAL_PORT ist nicht gesetzt. Beispiel: make prog SERIAL_PORT=COM5
	@exit /B 1
endif
	"$(ICEFUNPROG)" $(SERIAL_PORT) $(BIN)

clean:
	if exist $(BUILD_DIR) rmdir /S /Q $(BUILD_DIR)
