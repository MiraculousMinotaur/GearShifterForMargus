#!/usr/bin/env bash
set -e

# Determine repository root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
LIB_DIR="$REPO_ROOT/lib"

# Determine Arduino sketchbook libraries directory
if [ -d "$HOME/Arduino/libraries" ]; then
    ARDUINO_LIB_DIR="$HOME/Arduino/libraries"
elif [ -d "$HOME/Documents/Arduino/libraries" ]; then
    ARDUINO_LIB_DIR="$HOME/Documents/Arduino/libraries"
else
    ARDUINO_LIB_DIR="$HOME/Arduino/libraries"
    mkdir -p "$ARDUINO_LIB_DIR"
fi

LIBRARIES=(
    "ArduinoJoystickWithFFBLibrary"
    "Adafruit_ADS1X15"
    "Adafruit_BusIO"
    "Adafruit_MCP23017"
)

for lib in "${LIBRARIES[@]}"; do
    TARGET="$LIB_DIR/$lib"
    LINK="$ARDUINO_LIB_DIR/$lib"

    if [ -d "$TARGET" ]; then
        ln -sfn "$TARGET" "$LINK"
        echo "Linked: $lib -> $LINK"
    else
        echo "Warning: Source directory not found: $TARGET (did you run git submodule update --init --recursive?)" >&2
    fi
done
