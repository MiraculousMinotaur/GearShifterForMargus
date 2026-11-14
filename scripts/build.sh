#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

echo "==> Checking git submodules..."
if [ ! -f "lib/Adafruit_ADS1X15/Adafruit_ADS1X15.h" ] || [ ! -f "lib/ArduinoJoystickWithFFBLibrary/src/Joystick.h" ]; then
    echo "Submodules not initialized. Initializing..."
    git submodule update --init --recursive
fi

# Determine compiler toolchain
if command -v arduino-cli &>/dev/null; then
    echo "==> Building with arduino-cli..."
    arduino-cli compile main_sketch_GearStickForMargus "$@"
elif command -v pio &>/dev/null; then
    echo "==> Building with PlatformIO..."
    pio run "$@"
else
    echo "Error: Neither 'arduino-cli' nor 'pio' was found in PATH." >&2
    echo "" >&2
    echo "To install arduino-cli on Linux:" >&2
    echo "  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh" >&2
    echo "  arduino-cli core install arduino:avr" >&2
    echo "" >&2
    echo "Or install PlatformIO:" >&2
    echo "  pip install platformio" >&2
    exit 1
fi
