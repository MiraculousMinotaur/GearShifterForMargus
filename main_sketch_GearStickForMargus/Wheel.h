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
// Using pins 0 and 1 (INT2 and INT3 on ATmega32u4) for direct AVR hardware interrupt support
// This provides lower latency and reduced overhead compared to Arduino's attachInterrupt()
#define ENCODER_PIN_A 0  // INT2 hardware interrupt
#define ENCODER_PIN_B 1  // INT3 hardware interrupt
// Encoder Limits
#define ENCODER_MIN_VALUE -3600 // One Full rotation is 2400
#define ENCODER_MAX_VALUE 3600

// Expose position for diagnostics
extern volatile int32_t currentPosition;

#if FFB
#define MAX_FORCES 250 // Testing revealed Force MAX values is 250
#define MAX_ENDPOINT_FORCES 500
#define ENDSTOP_WIDTH_TICKS 100 // Width of the endstop deadband in encoder ticks
#define SELFCENTER_ON 0
#define SELFCENTER_FORCE 50 // Constant force applied toward center when FFB is inactive (tuning required)
#define SELFCENTER_DEADZONE 20 // Deadzone around center to prevent noise/oscillation (in encoder units)
void Handle_forces_Idle(void);
#endif

void Wheel_begin();
void Wheel_update(int32_t wheelValue);

volatile int32_t Get_CurrentPosition(void);

// Wheel computes target forces but does not directly control motor.
// All motor control flows through ACS712 via ACS712_setTargetFromForce().

// Debug report function (opaque pointer, actual type defined in DebugManager.h)
#if DEBUG
struct DebugTelemetry_t;
void Wheel_reportDebug(struct DebugTelemetry_t *tel);
#endif

#endif // WHEEL

#endif // WHEEL_H
