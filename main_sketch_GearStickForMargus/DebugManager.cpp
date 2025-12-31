#include "DebugManager.h"
#include "Config.h"
#include "ACS712Driver.h"
#include "Wheel.h"
#include "motor.h"
#if DEBUG
#include <Arduino.h>
#endif

void DebugManager_init(void)
{
  // No-op for now; Serial is setup in main when DEBUG enabled
}

void DebugManager_update(void)
{
#if DEBUG
  static char cmdBuf[48];
  static uint8_t cmdIdx = 0;
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (cmdIdx > 0) {
        cmdBuf[cmdIdx] = '\0';
        // Process command
        if (strcasecmp(cmdBuf, "S") == 0) {
          Serial.print("ENC:"); Serial.print(currentPosition);
          Serial.print(" MotorLast:"); Serial.println(Motor_getLastForce());
          // Ask ACS712 to dump status too
          ACS712_processCommand("S");
        }
        else if (strcasecmp(cmdBuf, "ENC") == 0) {
          Serial.print("ENC:"); Serial.println(currentPosition);
        }
        else if (strncasecmp(cmdBuf, "M:", 2) == 0) {
          int v = atoi(cmdBuf + 2);
          Motor_set(v);
          Serial.print("OK M:"); Serial.println(v);
        }
        else if (strncasecmp(cmdBuf, "T:", 2) == 0) {
          ACS712_processCommand(cmdBuf);
        }
        else {
          Serial.print("UNK CMD: "); Serial.println(cmdBuf);
        }
        cmdIdx = 0;
      }
    } else {
      if (cmdIdx < (sizeof(cmdBuf) - 1)) cmdBuf[cmdIdx++] = c;
    }
  }
#endif
}
