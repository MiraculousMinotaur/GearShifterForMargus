#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "FixedPoint.h"

void Motor_init(void);
void Motor_set(int16_t pwmValue);

// Diagnostic
int16_t Motor_getLastPWMTarget(void);

#endif // MOTOR_H
