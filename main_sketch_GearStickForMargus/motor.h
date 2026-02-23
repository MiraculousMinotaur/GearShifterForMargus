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

void Motor_init(void);
void Motor_set(int16_t pwmValue);

// Diagnostic
int16_t Motor_getLastPWMTarget(void);

#endif // MOTOR_H
