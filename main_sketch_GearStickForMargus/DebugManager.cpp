#include "DebugManager.h"
#include "Config.h"
#if DEBUG
#include "ACS712Driver.h"
#include "Wheel.h"
#include "Pedals.h"
#include "Gears.h"
#include <Arduino.h>

// Telemetry buffer
static DebugTelemetry_t telemetryBuffer = {0};

// CSV output timing (in milliseconds)
static const uint32_t CSV_OUTPUT_INTERVAL_MS = 500;  // Output CSV every 500ms
static uint32_t lastCsvTime = 0;

// ===== Module ReportDebug() function declarations =====
// These must be defined in each module. They populate the telemetry buffer.
#if WHEEL
extern void Wheel_reportDebug(DebugTelemetry_t *tel);
#endif
#if FFB
extern void ACS712_reportDebug(DebugTelemetry_t *tel);
#endif
#if PEDALS
extern void Pedals_reportDebug(DebugTelemetry_t *tel);
#endif
#if GEARS
extern void Gears_reportDebug(DebugTelemetry_t *tel);
#endif

void DebugManager_init(void)
{
  // No-op for now; Serial is setup in main when DEBUG enabled
  lastCsvTime = millis();
}

const DebugTelemetry_t* DebugManager_getTelemetry(void)
{
  return &telemetryBuffer;
}

// ===== CSV Output =====
static void outputCSVTelemetry(void)
{
  // Format: WHEEL,<pos>,ACS,<raw>,<delta>,<target>,<duty>,<kp>,<ki>,<kd>,ACCEL,<val>,BRAKE,<val>,CLUTCH,<val>,MCP,<0b1010101010101010>
  Serial.print("WHEEL,");
  Serial.print(telemetryBuffer.wheel_position);
  Serial.print(",ACS,");
  Serial.print((int)telemetryBuffer.acs_raw_adc);
  Serial.print(",");
  Serial.print(telemetryBuffer.acs_delta);
  Serial.print(",");
  Serial.print((int)telemetryBuffer.acs_target);
  Serial.print(",");
  Serial.print(telemetryBuffer.acs_current_duty);
  Serial.print(",");
  Serial.print(telemetryBuffer.acs_kp_q8);
  Serial.print(",");
  Serial.print(telemetryBuffer.acs_ki_q8);
  Serial.print(",");
  Serial.print(telemetryBuffer.acs_kd_q8);
  Serial.print(",PEDALS,");
  Serial.print(telemetryBuffer.pedals_accel);
  Serial.print(",");
  Serial.print(telemetryBuffer.pedals_brake);
  Serial.print(",");
  Serial.print(telemetryBuffer.pedals_clutch);
  Serial.print(",MCP,0x");
  Serial.print(telemetryBuffer.gears_gpio, BIN);
  Serial.println();
}

void DebugManager_update(void)
{
  // === Collect telemetry from all modules ===
  #if WHEEL
  Wheel_reportDebug(&telemetryBuffer);
  #endif
  
  #if FFB
  ACS712_reportDebug(&telemetryBuffer);
  #endif
  
  #if PEDALS
  Pedals_reportDebug(&telemetryBuffer);
  #endif
  
  #if GEARS
  Gears_reportDebug(&telemetryBuffer);
  #endif

  // === Periodic CSV output ===
  uint32_t now = millis();
  if ((now - lastCsvTime) >= CSV_OUTPUT_INTERVAL_MS) {
    lastCsvTime = now;
    outputCSVTelemetry();
  }
}
#endif // DEBUG
