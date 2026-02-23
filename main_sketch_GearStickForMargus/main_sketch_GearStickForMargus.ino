// 32u4 based board
// Clean main sketch: delegates behavior to modular files (Gears, Wheel, Pedals)

#include "Config.h"
#include "Utils.h"
#include "Gears.h"
#include "Wheel.h"
#include "ACS712Driver.h"
#include "Pedals.h"
#include "Scheduler.h"
#if DEBUG
#include "DebugManager.h"
#endif
#include <Wire.h>

// Joystick object is defined in JoystickGlue.cpp and declared extern in Config.h

void setup() {
#if DEBUG
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 5000) { ; }
  Serial.println("Serial started");
  DebugManager_init();
#endif

  // ========== I2C Initialization (for ADS1115 pedals and MCP23017 gears) ==========
  // Enable external I2C device power supply before initializing I2C bus
  pinMode(I2C_POWER_ENABLE_PIN, OUTPUT);
#if I2C_POWER_ENABLE_ACTIVE_HIGH
  digitalWrite(I2C_POWER_ENABLE_PIN, HIGH);   // Enable power
#else
  digitalWrite(I2C_POWER_ENABLE_PIN, LOW);    // Enable power (active-low)
#endif
  delay(I2C_POWER_STABILIZE_MS);  // Wait for external devices to stabilize
  
  // Initialize I2C bus (Wire library)
  Wire.begin();
  Wire.setClock(400000); // Set I2C clock to 400kHz for faster communication with ADS1115 and MCP23017
  // Initialize MCP23017 for gear inputs
  #if GEARS
  mcp.begin_I2C(MCP23017_I2C_ADDR);
  #endif
  
  // Initialize ADS1115 for pedal analog inputs
  #if PEDALS
  ads.begin(ADS1115_I2C_ADDR);
  ads.setGain(ADS1115_GAIN);
  ads.setDataRate(ADS1115_DATARATE);
  
  // Conversion is started in Pedals_Begin() after calibration values are set
  #endif
// Initialize Joystick Library, must be called after Serial for debug prints and before modules which use Joystick
  Joystick.begin(false); // Don't auto-send state; we'll call Joystick.sendState() manually in the main loop after updates
  // ========== Module initializations ==========
  // Initialize Wheel before ACS712 so encoder and FFB setup is complete
#if WHEEL
  Wheel_begin();
#endif
#if FFB
  ACS712_begin();
  ACS712_enable(false); // Start with motor disabled; enable when FFB becomes active
#endif
#if PEDALS
  Pedals_begin();
#endif
#if GEARS
  Gears_begin();
#endif
  // Start the 1ms scheduler and debug manager
  Scheduler_start();

#if FFB
  delay(1000);
  ACS712_calibrateZero();
#endif

}

ISR(TIMER3_COMPA_vect){
  scheduler_ms_flag = 1;//Limit other actions to less then 1Khz just in case.
  uint16_t currentSum = 0;
  uint16_t currentCount = 0;
  // ACS712_snapshotAndClear is non-atomic by design; perform snapshot inside this interrupt-disabled section
  ACS712_snapshotAndClear(&currentSum, &currentCount);
  sei();
  // Apply averaged ADC value if samples were collected
  if (currentCount > 0)
  {
    uint16_t avg = currentSum / currentCount;
    ACS712_setLastADC(avg);
  }
  ACS712_update();
}

void loop() 
{
  // Module updates are scheduled by the 1ms Scheduler (see Scheduler_start and scheduler flag handling).
  // Scheduler-driven tasks triggered from Timer3 (1ms tick)
  static uint8_t schedCounter = 0;
  if (scheduler_ms_flag)
  {
    scheduler_ms_flag = 0;
    // 1.2) Every other loop: request FFB/USB processing
    if (schedCounter & 1)
    {
      Joystick.getUSBPID();
#if WHEEL
      Wheel_update();
#endif
    }
    
    // 1.3) Every odd tick: alternate pedals/gears reads (offset from USBPID which runs on even ticks)
    if (schedCounter > 4)
    {
      static uint8_t readPedals = 4;
      if (readPedals)
      {
        #if PEDALS
        Pedals_update();
        #endif
        readPedals--;
      }
      else
      {
        #if GEARS
        Gears_update();
        #endif
        readPedals = 4; // reset to read pedals for the next 4 cycles
      }
      schedCounter = 0;
    }

    // Wheel update runs each scheduler cycle to update axis and apply FFB/motor targets
    // 1.4) Debug manager at end of cycle
  #if DEBUG
    DebugManager_update();
  #endif
    schedCounter++;
  }
  Joystick.sendState(); // Send the current joystick state to the host computer; must be called regularly to ensure timely updates
}
