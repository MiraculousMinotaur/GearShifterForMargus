#ifndef WHEEL_H
#define WHEEL_H

#include <Arduino.h>
#include <Joystick.h>
#include "Config.h"
#include "Utils.h"

#if WHEEL

// Encoder Pins
#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3
// Encoder Limits
#define ENCODER_MIN_VALUE -3600 // One Full rotation is 2400
#define ENCODER_MAX_VALUE 3600

#if FFB
// Motor Pins
#define MOTOR_PIN_A 9
#define MOTOR_PIN_B 10
#define MOTOR_PIN_ENABLE A0
// Motor Limits
#define MAX_PWM 125 // Going full 255 has higher chance to burn the motor
#define MAX_CENTERING_PWM 50 // Going full 255 has higher chance to burn the motor
#define MAX_FORCES 250 // Testing revealed Force MAX values is 250
#endif

void Wheel_begin();
void Wheel_update();

#endif // WHEEL

#endif // WHEEL_H
