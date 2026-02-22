#ifndef GEARS_H
#define GEARS_H

#include <Arduino.h>
#include <Joystick.h>
#include "Config.h"

#if GEARS

#include <Adafruit_MCP23X17.h>

// MCP23017 Pin Assignments for Gear Inputs
// NOTE: These are MCP23017 logical pin indices (GPA0-GPA7 = 0-7, GPB0-GPB7 = 8-15)
// Active-low logic is preserved: when bit is 0 (pin LOW), gear is active
// TODO: verify with wiring and provide documentation on expected pin connections for each gear switch
#define SIX_WAY_PIN_1 (MCP_GEAR_SIXWAY_FIRST_PIN + 0)  // MCP pin 10 (GPB2)
#define SIX_WAY_PIN_2 (MCP_GEAR_SIXWAY_FIRST_PIN + 1)  // MCP pin 11 (GPB3)
#define SIX_WAY_PIN_3 (MCP_GEAR_SIXWAY_FIRST_PIN + 2)  // MCP pin 12 (GPB4)
#define SIX_WAY_PIN_4 (MCP_GEAR_SIXWAY_FIRST_PIN + 3)  // MCP pin 13 (GPB5)
#define SIX_WAY_PIN_5 (MCP_GEAR_SIXWAY_FIRST_PIN + 4)  // MCP pin 14 (GPB6)
#define SIX_WAY_PIN_6 (MCP_GEAR_SIXWAY_FIRST_PIN + 5)  // MCP pin 15 (GPB7)

// Mode switch Pins
#define MODE_PIN_1 (MCP_GEAR_MODE_FIRST_PIN + 0)       // MCP pin 0 (GPA0) -> Reverse
#define MODE_PIN_2 (MCP_GEAR_MODE_FIRST_PIN + 1)       // MCP pin 1 (GPA1) -> Low_Range

// Impulse Switch Pins
#define LOWER_IMPULSE_PIN   (MCP_GEAR_IMPULSE_FIRST_PIN + 0)  // MCP pin 6 (GPA6)
#define HIGHER_IMPULSE_PIN  (MCP_GEAR_IMPULSE_FIRST_PIN + 1)  // MCP pin 7 (GPA7)

#define MAIN_STICK_SIZE 6

enum Buttons_e {
  NORMAL_1 = 0,
  NORMAL_2,
  NORMAL_3,
  NORMAL_4,
  NORMAL_5,
  NORMAL_6,
  REVERSE,
  HIGH_RANGE,
  IMPULSE_1,
  IMPULSE_2,
  LAST_BUTTON
};

// Extern MCP23017 instance (defined in Gears.cpp)
extern Adafruit_MCP23X17 mcp;

void Gears_begin();
void Gears_update();

// Debug report function (opaque pointer, actual type defined in DebugManager.h)
#if DEBUG
struct DebugTelemetry_t;
void Gears_reportDebug(struct DebugTelemetry_t *tel);
#endif

#endif // GEARS

#endif // GEARS_H
