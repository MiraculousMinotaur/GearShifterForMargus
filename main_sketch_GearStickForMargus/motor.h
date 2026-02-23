#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "FixedPoint.h"

// Motor Pins
#define MOTOR_PIN_A 9
#define MOTOR_PIN_B 10
#define MOTOR_PIN_ENABLE 6

// Motor Limits
#define PWM_TIMER_MAX 0x2FF // 16-bit timer with ICR1 TOP, corresponds to ~21kHz PWM frequency
#define MAX_PWM 600 // Going full 255 has higher chance to burn the motor
#define CLOCKWISE_BIAS(x) (x + (x>>3) + (x>>4)) //1.1875 times multiplication
#define SLEW_STEP 5
const int16_t MAX_PWM_WITH_SLEW = MAX_PWM / SLEW_STEP * SLEW_STEP; // max target that can be reached with slew rate from 0 to MAX_PWM


void Motor_init(void);
void Motor_set(int16_t pwmValue);

// Diagnostic
int16_t Motor_getLastPWMTarget(void);

#endif // MOTOR_H
