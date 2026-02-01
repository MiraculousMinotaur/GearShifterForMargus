#ifndef GEARS_H
#define GEARS_H

#include <Arduino.h>
#include <Joystick.h>
#include "Config.h"

#if GEARS

// Six way pins
// TODO: Pin assignments using A5/A4 may conflict with ACS712 pins (ACS712_PIN_SENSE/ACS712_PIN_POWER). Consider centralizing pin definitions in `Config.h` and providing alternative pins.
#define SIX_WAY_PIN_1 5
#define SIX_WAY_PIN_2 A5
#define SIX_WAY_PIN_3 6
#define SIX_WAY_PIN_4 A4
#define SIX_WAY_PIN_5 7
#define SIX_WAY_PIN_6 A3

// Mode switch Pins
#define MODE_PIN_1 10 //Reverse
#define MODE_PIN_2 11 //Low_Range

// Impulse Switch Pins
#define LOWER_IMPULSE_PIN   9
#define HIGHER_IMPULSE_PIN 8

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

void Gears_begin();
void Gears_update();

#endif // GEARS

#endif // GEARS_H
