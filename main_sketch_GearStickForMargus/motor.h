#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

void Motor_init(void);
void Motor_set(int force);

// legacy name kept for compatibility
void setMotor(int force);

#endif // MOTOR_H
