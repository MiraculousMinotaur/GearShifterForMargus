#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

void Motor_init(void);
void Motor_set(int force);

// legacy name kept for compatibility
void setMotor(int force);

// FFB helpers
int Motor_rampForceToPWM(int rawForce);
int Motor_computeEndpointPWM(int currentPosition);
void Motor_selfCenter(int wheelOutput);

// Diagnostic
int Motor_getLastForce(void);

#endif // MOTOR_H
