#include "Config.h"

// Provide the global Joystick object used by the modules and the sketch.
// The size of the button report is derived from the GEARS flag in Config.h
#if GEARS
const uint8_t LAST_BUTTON = 10;
#else
const uint8_t LAST_BUTTON = 0;
#endif

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
  LAST_BUTTON, 0,                  // Button Count, Hat Switch Count
  true, true, true,     // X and Y, but no Z Axis
  true, true, false,   //  Rx, Ry, no Rz
  false, false,          // No rudder or throttle
  false, false, false);    // No accelerator, brake, or steering
