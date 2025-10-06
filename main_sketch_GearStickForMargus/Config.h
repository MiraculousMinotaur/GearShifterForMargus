#ifndef CONFIG_H
#define CONFIG_H

// Feature flags -- centralized so .cpp/.h files can see the same values as the .ino
#define DEBUG 0
#define PEDALS 0
#define GEARS 0 // TODO: conflicts with wheel pins for now.
#define WHEEL 1
#if WHEEL
#define FFB 1 // FFB currently only effects steering
#else
#define FFB 0
#endif

#include <Joystick.h>
// Make the global Joystick object accessible from .cpp modules
extern Joystick_ Joystick;

#endif // CONFIG_H
