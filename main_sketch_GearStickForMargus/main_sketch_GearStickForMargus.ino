// 32u4 based board
// Clean main sketch: delegates behavior to modular files (Gears, Wheel, Pedals)

#include "Config.h"
#include "Utils.h"
#include "Gears.h"
#include "Wheel.h"
#include "ACS712Driver.h"
#include "Pedals.h"

#if DEBUG
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif

// Joystick object is defined in JoystickGlue.cpp and declared extern in Config.h

void setup() {
#if DEBUG
  Serial.begin(9600);
#endif

  // Module initializations
#if PEDALS
  Pedals_begin();
#endif

#if GEARS
  Gears_begin();
#endif

#if WHEEL
  Wheel_begin();
#endif

  // ACS712 current-sensor driver
  ACS712_begin();

  // Initialize Joystick Library
  Joystick.begin(true);
}

#if FFB
ISR(TIMER3_COMPA_vect){
  Joystick.getUSBPID();
  ACS712_onTimerTick_ISR();
}
#endif

void loop() 
{
#if DEBUG
  delay(100);
#endif

  // Module updates
#if PEDALS
  Pedals_update();
#endif

#if GEARS
  Gears_update();
#endif

#if WHEEL
  Wheel_update();
#endif

  // ACS712 driver background sampling & command processing
  ACS712_backgroundTask();

  // Apply control if timer ticked (non-blocking)
  ACS712_update();
}
