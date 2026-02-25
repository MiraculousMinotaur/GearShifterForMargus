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
#define WATCHDOG_TIMER 20 //ms
#define SLOW_SCHEDULER_TIMER 5 //ms //must be smaller then WATCHDOG_TIMER
volatile int8_t SchedulerTimer = WATCHDOG_TIMER;

ISR(TIMER3_COMPA_vect){
  SchedulerTimer--;
  uint16_t currentSum = 0;
  uint16_t currentCount = 0;
  // ACS712_snapshotAndClear is non-atomic by design; perform snapshot inside this interrupt-disabled section
  ACS712_snapshotAndClear(&currentSum, &currentCount);
  int32_t wheelValue = Get_CurrentPosition();
  sei();
  // Apply averaged ADC value if samples were collected
  if (currentCount > 0){
    uint16_t avg = currentSum / currentCount;
    ACS712_setLastADC(avg);
  }
  Wheel_update(wheelValue);
  ACS712_update();
}

void loop() 
{
  if (SchedulerTimer <= WATCHDOG_TIMER-SLOW_SCHEDULER_TIMER)
  {
    Joystick.getUSBPID(); // Regularly check for USB PID data to update FFB effects; runs every scheduler cycle (1ms)
    Handle_forces_Idle(); // Long flaot based effect calculations
    Joystick.sendState(); // Send the current joystick state to the host computer; must be called regularly to ensure timely updates
    static bool readGears = false;
    #if PEDALS
    if(get_Ads_State() == IDLE){readGears = true;} // Only read gears when ADS is idle to prevent I2C conflicts; this means gears are only updated every ~60ms when pedals are active, but that's acceptable for a gear stick
    Pedals_update();
    #endif
    if(readGears)
    {
      #if GEARS
      Gears_update();
      #endif
      readGears = false;
    }
    SchedulerTimer = WATCHDOG_TIMER;
  }
#if DEBUG
    DebugManager_update();
#endif
}
