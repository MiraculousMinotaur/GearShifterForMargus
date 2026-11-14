# Makefile for GearShifterForMargus
# Cross-platform build automation for Linux, macOS, and Windows

SKETCH_DIR  := main_sketch_GearStickForMargus
SKETCH      := $(SKETCH_DIR)/main_sketch_GearStickForMargus.ino
FQBN        := arduino:avr:leonardo
LIB_DIR     := lib
BUILD_DIR   := build
PORT        ?= /dev/ttyACM0
BAUD        ?= 115200

# Detect available build tools in PATH
ARDUINO_CLI := $(shell command -v arduino-cli 2>/dev/null)
PIO         := $(shell command -v pio 2>/dev/null)

.PHONY: all build compile upload monitor clean submodules setup help

all: build

help:
	@echo "GearShifterForMargus Build System"
	@echo "=================================="
	@echo "Usage:"
	@echo "  make              - Ensure submodules exist and compile the sketch"
	@echo "  make upload       - Compile and upload to board (PORT=$(PORT))"
	@echo "  make monitor      - Open serial monitor (PORT=$(PORT), BAUD=$(BAUD))"
	@echo "  make submodules   - Initialize and recursively update git submodules"
	@echo "  make setup        - Install Arduino AVR core (for arduino-cli)"
	@echo "  make clean        - Remove build artifacts"

submodules:
	@if [ ! -f "$(LIB_DIR)/Adafruit_ADS1X15/Adafruit_ADS1X15.h" ] || [ ! -f "$(LIB_DIR)/ArduinoJoystickWithFFBLibrary/src/Joystick.h" ]; then \
		echo "==> Initializing and updating submodules..."; \
		git submodule update --init --recursive; \
	fi

setup:
ifdef ARDUINO_CLI
	@echo "==> Updating arduino-cli core index..."
	$(ARDUINO_CLI) core update-index
	@echo "==> Installing arduino:avr core..."
	$(ARDUINO_CLI) core install arduino:avr
else
	@echo "arduino-cli not found in PATH."
	@echo "To install on Linux:"
	@echo "  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh"
	@exit 1
endif

build: submodules
ifdef ARDUINO_CLI
	@echo "==> Building with arduino-cli ($(FQBN))..."
	$(ARDUINO_CLI) compile --build-path $(BUILD_DIR) $(SKETCH_DIR)
else ifdef PIO
	@echo "==> Building with PlatformIO..."
	$(PIO) run
else
	@echo "Error: Neither 'arduino-cli' nor 'pio' (PlatformIO) was found in PATH."
	@echo ""
	@echo "Install arduino-cli (recommended for Linux):"
	@echo "  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh"
	@echo "  make setup"
	@echo ""
	@echo "Or install PlatformIO:"
	@echo "  pip install platformio"
	@exit 1
endif

compile: build

upload: build
ifdef ARDUINO_CLI
	@echo "==> Uploading to $(PORT) using arduino-cli..."
	$(ARDUINO_CLI) upload -p $(PORT) --fqBN $(FQBN) --input-dir $(BUILD_DIR) $(SKETCH_DIR)
else ifdef PIO
	@echo "==> Uploading to $(PORT) using PlatformIO..."
	$(PIO) run -t upload --upload-port $(PORT)
endif

monitor:
ifdef ARDUINO_CLI
	$(ARDUINO_CLI) monitor -p $(PORT) -c baudrate=$(BAUD)
else ifdef PIO
	$(PIO) device monitor -b $(BAUD) -p $(PORT)
else
	@echo "Error: Neither arduino-cli nor pio found to monitor $(PORT)."
	@exit 1
endif

clean:
	rm -rf $(BUILD_DIR) .pio
