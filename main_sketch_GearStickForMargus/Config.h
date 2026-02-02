#ifndef CONFIG_H
#define CONFIG_H

// Feature flags -- centralized so .cpp/.h files can see the same values as the .ino
#define DEBUG 1
#define PEDALS 0
#define GEARS 0 // TODO: Fix pin conflict between GEARS and WHEEL. Centralize pin definitions and provide alternative pin assignments or disable GEARS by default. Action: add `ACS712_PIN_*` defines and document wiring; verify no pin overlap.
#define WHEEL 1
#if WHEEL
#define FFB 1 // FFB currently only effects steering
#else
#define FFB 0
#endif

// Default pin assignments (can be overridden per-board)
// TODO: Verify these pins don't conflict with GEARS/WHEEL. If conflict exists, provide alternatives in Config.h or a board-specific header.
#define ACS712_PIN_SENSE A5
#define ACS712_PIN_POWER A4

#include <Joystick.h>
// Make the global Joystick object accessible from .cpp modules
extern Joystick_ Joystick;

// Endpoint enforcement tuning (used by Wheel.cpp)
// Higher values make the endpoint enforcement engage sooner (smaller breach)
// Set to 24 so that ~1/8 of a full rotation (~300 encoder clicks) will reach MAX_PWM
#define ENDPOINT_BAND 24

#endif // CONFIG_H
