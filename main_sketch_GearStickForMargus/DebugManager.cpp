#include "DebugManager.h"
#include "Config.h"
#if DEBUG
#include "ACS712Driver.h"
#include "Wheel.h"
#include "Pedals.h"
#include "Gears.h"
#include "motor.h"
#include <Arduino.h>
#include <string.h>

// Telemetry buffer
static DebugTelemetry_t telemetryBuffer = {0};

// Serial command buffer
static char cmdBuf[64];
static uint8_t cmdIdx = 0;

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
  // Format: WHEEL,<pos>,ACS,<raw>,<delta>,<target>,<duty>,<kp>,<ki>,<kd>,ACCEL,<val>,BRAKE,<val>,CLUTCH,<val>,VREF,<val>,MCP,<0xHHHH>
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
  Serial.print(",");
  Serial.print(telemetryBuffer.pedals_vref);
  Serial.print(",MCP,0x");
  Serial.print(telemetryBuffer.gears_gpio, HEX);
  Serial.println();
}

// ===== Command Processing =====
static void processCommand(const char *cmd)
{
  // Commands:
  // SENSORS - output current telemetry
  // MOTOR:START - enable motor control
  // MOTOR:STOP - disable motor control
  // MOTOR:CURRENT:<value> - set ACS712 target ADC value (FFB only)
  // MOTOR:SELFCENTER:ON - enable self-centering (FFB only)
  // MOTOR:SELFCENTER:OFF - disable self-centering (FFB only)
  // PID:Kp=<value> - set ACS712 Kp (Q8, FFB only)
  // PID:Ki=<value> - set ACS712 Ki (Q8, FFB only)
  // PID:Kd=<value> - set ACS712 Kd (Q8, FFB only)
  
  if (strcasecmp(cmd, "SENSORS") == 0) {
    // Output telemetry immediately
    outputCSVTelemetry();
  }
#if FFB
  else if (strncasecmp(cmd, "MOTOR:", 6) == 0) {
    const char *subcmd = cmd + 6;
    if (strcasecmp(subcmd, "START") == 0) {
      ACS712_enable(true);
      Serial.println("OK MOTOR:START");
    }
    else if (strcasecmp(subcmd, "STOP") == 0) {
      ACS712_enable(false);
      Motor_set(0);
      Serial.println("OK MOTOR:STOP");
    }
    else if (strncasecmp(subcmd, "CURRENT:", 8) == 0) {
      int adcTarget = atoi(subcmd + 8);
      ACS712_setTargetA(adcTarget);
      Serial.print("OK MOTOR:CURRENT:");
      Serial.println(adcTarget);
    }
    else if (strncasecmp(subcmd, "SELFCENTER:", 11) == 0) {
      const char *onoff = subcmd + 11;
      if (strcasecmp(onoff, "ON") == 0) {
        Serial.println("OK MOTOR:SELFCENTER:ON (not yet implemented)");
      }
      else if (strcasecmp(onoff, "OFF") == 0) {
        Serial.println("OK MOTOR:SELFCENTER:OFF (not yet implemented)");
      }
      else {
        Serial.println("ERR MOTOR:SELFCENTER invalid");
      }
    }
    else {
      Serial.print("ERR MOTOR unknown: ");
      Serial.println(subcmd);
    }
  }
  else if (strncasecmp(cmd, "PID:", 4) == 0) {
    const char *param = cmd + 4;
    // Parse format: "Kp=<value>" or "Ki=<value>" or "Kd=<value>"
    
    if (strncasecmp(param, "Kp=", 3) == 0) {
      int16_t value = (int16_t)atoi(param + 3);
      ACS712_setGains_q8(value, -1, -1);
      Serial.print("OK PID:Kp=");
      Serial.println(value);
    }
    else if (strncasecmp(param, "Ki=", 3) == 0) {
      int16_t value = (int16_t)atoi(param + 3);
      ACS712_setGains_q8(-1, value, -1);
      Serial.print("OK PID:Ki=");
      Serial.println(value);
    }
    else if (strncasecmp(param, "Kd=", 3) == 0) {
      int16_t value = (int16_t)atoi(param + 3);
      ACS712_setGains_q8(-1, -1, value);
      Serial.print("OK PID:Kd=");
      Serial.println(value);
    }
    else {
      Serial.print("ERR PID unknown: ");
      Serial.println(param);
    }
  }
#endif
  else {
    Serial.print("ERR UNK CMD: ");
    Serial.println(cmd);
  }
}

void DebugManager_update(void)
{
  // === Handle serial input (non-blocking) ===
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (cmdIdx > 0) {
        cmdBuf[cmdIdx] = '\0';
        processCommand(cmdBuf);
        cmdIdx = 0;
      }
    } else {
      if (cmdIdx < (sizeof(cmdBuf) - 1)) {
        cmdBuf[cmdIdx++] = c;
      }
    }
  }

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
