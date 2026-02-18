#ifndef ACS712DRIVER_H
#define ACS712DRIVER_H

#include <Arduino.h>

// Note: Pin constants (e.g., ACS712_PIN_SENSE, ACS712_PIN_POWER) are defined in `Config.h`.
// Public API
void ACS712_begin();
void ACS712_update();           // call regularly in loop(), does control when timer ticked

// Debug task (background work can be moved to DebugManager) - optional
void ACS712_DebugTask();
// ADC control API for free-running sampling
void setupADC(uint8_t channel);
void startADC();
void stopADC();
// Non-atomic snapshot helper: copies and clears ISR accumulators. Caller must protect with noInterrupts()/interrupts() if atomicity required.
void ACS712_snapshotAndClear(uint16_t *sum, uint16_t *count);
// Set lastADC from caller (used after atomic snapshot)
void ACS712_setLastADC(uint16_t v);
void ACS712_setTargetA(int16_t adcValue);
void ACS712_setTargetFromForce(int16_t force); // force in -MAX_FORCES..MAX_FORCES maps to ADC target around zeroADC
void ACS712_enable(bool en);
bool ACS712_isEnabled();
// Set gains in Q8 fixed-point: value * 256. e.g. kp=1.0 -> 256
void ACS712_setGains_q8(int16_t kp_q8, int16_t ki_q8, int16_t kd_q8);
void ACS712_calibrateZero();
void ACS712_processCommand(const char *cmd);

// Lightweight ISR hook (called from TIMER3 COMPA ISR) - keep very short
void ACS712_onTimerTick_ISR();

#endif // ACS712DRIVER_H