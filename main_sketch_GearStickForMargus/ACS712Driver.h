#ifndef ACS712DRIVER_H
#define ACS712DRIVER_H

#include <Arduino.h>

// Public API
void ACS712_begin();
void ACS712_update();           // call regularly in loop(), does control when timer ticked
void ACS712_backgroundTask();   // call in loop() frequently to perform non-blocking sampling and serial parsing
void ACS712_setTargetA(float amps);
void ACS712_setTargetFromForce(int8_t force); // force in -255..255 maps to -MAX_AMPS..MAX_AMPS
void ACS712_enable(bool en);
bool ACS712_isEnabled();
void ACS712_setGains(float kp, float ki, float kd);
void ACS712_calibrateZero();
void ACS712_processCommand(const char *cmd);

// Lightweight ISR hook (called from TIMER3 COMPA ISR) - keep very short
void ACS712_onTimerTick_ISR();

#endif // ACS712DRIVER_H