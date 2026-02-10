#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "FixedPoint.h"

// PID Control Gains (Q8 fixed-point)
#define MOTOR_SELFCENTER_KP_Q8 (int16_t)(0.05 * SCALE_Q8)  // ~13
#define MOTOR_SELFCENTER_KI_Q8 (int16_t)(0.05 * SCALE_Q8)  // ~13
#define MOTOR_SELFCENTER_KD_Q8 (int16_t)(0.1  * SCALE_Q8)  // ~26

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
