#ifndef WHEEL_H
#define WHEEL_H

#include <Arduino.h>
#include <Joystick.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Config.h"
#include "Utils.h"

#if WHEEL

// Encoder Pins
// moved to use Serial1 / INT-capable pins per user
#define ENCODER_PIN_A 0  // Note: if only 1 interrupt is to be used PIN 7 and 6 are also options as 7 supports external interrupts.
#define ENCODER_PIN_B 1
// Encoder Limits
#define ENCODER_MIN_VALUE -3600 // One Full rotation is 2400
#define ENCODER_MAX_VALUE 3600

// Expose position for diagnostics
extern volatile int32_t currentPosition;

#if FFB
// Motor Pins
#define MOTOR_PIN_A 9
#define MOTOR_PIN_B 10
#define MOTOR_PIN_ENABLE A0

#define MAX_FORCES 250 // Testing revealed Force MAX values is 250
#endif

void Wheel_begin();
void Wheel_update();

// Wheel computes target forces but does not directly control motor.
// All motor control flows through ACS712 via ACS712_setTargetFromForce().

// Debug report function (opaque pointer, actual type defined in DebugManager.h)
#if DEBUG
struct DebugTelemetry_t;
void Wheel_reportDebug(struct DebugTelemetry_t *tel);
#endif

#endif // WHEEL

#endif // WHEEL_H
