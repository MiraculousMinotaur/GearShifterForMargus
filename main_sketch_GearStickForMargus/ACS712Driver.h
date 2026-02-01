#ifndef ACS712DRIVER_H
#define ACS712DRIVER_H

#include <Arduino.h>

// Note: Pin constants (e.g., ACS712_PIN_SENSE, ACS712_PIN_POWER) are defined in `Config.h`.
// Public API
void ACS712_begin();
void ACS712_update();           // call regularly in loop(), does control when timer ticked
void ACS712_backgroundTask();   // call in loop() frequently to perform non-blocking sampling and serial parsing
void ACS712_setTargetA(int adcValue);
void ACS712_setTargetFromForce(int force); // force in -MAX_FORCES..MAX_FORCES maps to ADC target around zeroADC
void ACS712_enable(bool en);
bool ACS712_isEnabled();
// Set gains in Q8 fixed-point: value * 256. e.g. kp=1.0 -> 256
void ACS712_setGains_q8(int16_t kp_q8, int16_t ki_q8, int16_t kd_q8);
void ACS712_calibrateZero();
void ACS712_processCommand(const char *cmd);

// Lightweight ISR hook (called from TIMER3 COMPA ISR) - keep very short
void ACS712_onTimerTick_ISR();

#endif // ACS712DRIVER_H