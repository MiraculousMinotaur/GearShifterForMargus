#ifndef DEBUGMANAGER_H
#define DEBUGMANAGER_H

#include <stdint.h>

// ===== Debug Telemetry Buffer =====
// Holds sensor data collected from all modules for CSV output
struct DebugTelemetry_t {
  int32_t wheel_position;           // Encoder position
  uint16_t acs_raw_adc;             // Raw ADC reading (0-1023)
  int16_t acs_delta;                // Delta from zero (signed)
  uint16_t acs_target;              // Target ADC value
  int16_t acs_current_duty;         // Last motor duty/force sent by ACS controller
  int16_t acs_kp_q8;                // PID Kp (in Q8 fixed point)
  int16_t acs_ki_q8;                // PID Ki (in Q8 fixed point)
  int16_t acs_kd_q8;                // PID Kd (in Q8 fixed point)
  int16_t pedals_accel;             // Accelerator raw value
  int16_t pedals_brake;             // Brake raw value
  int16_t pedals_clutch;            // Clutch raw value
  int16_t pedals_vref;              // Reference voltage raw value
  uint16_t gears_gpio;              // MCP GPIO byte (all 16 bits)
};

typedef struct DebugTelemetry_t DebugTelemetry_t;

// ===== API Functions =====
#if DEBUG
void DebugManager_init(void);
void DebugManager_update(void);

// Get current telemetry buffer (read-only)
const DebugTelemetry_t* DebugManager_getTelemetry(void);
#endif // DEBUG

#endif // DEBUGMANAGER_H
