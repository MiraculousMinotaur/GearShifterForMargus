// 32u4 based board
// Clean main sketch: delegates behavior to modular files (Gears, Wheel, Pedals)

#include "Config.h"
#include "Utils.h"
#include "Gears.h"
#include "Wheel.h"
#include "ACS712Driver.h"
#include "Pedals.h"
#include "Scheduler.h"
#include "DebugManager.h"

#if DEBUG
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif

// Joystick object is defined in JoystickGlue.cpp and declared extern in Config.h

void setup() {
#if DEBUG
  Serial.begin(9600);
  DebugManager_init();
#endif

  // Module initializations


#if WHEEL
  Wheel_begin();
#endif
#if FFB
  ACS712_begin();
#endif
#if PEDALS
  Pedals_begin();
#endif

#if GEARS
  Gears_begin();
#endif


  // Initialize Joystick Library
  Joystick.begin(true);

  // Start the 1ms scheduler and debug manager
  Scheduler_start();

}


ISR(TIMER3_COMPA_vect){
  // Minimal ISR: set scheduler flag only. Heavy work happens in main context.
  scheduler_ms_flag = 1;
}

void loop() 
{
#if DEBUG
  delay(100);
#endif

  // Module updates are scheduled by the 1ms Scheduler (see Scheduler_start and scheduler flag handling).
  // Scheduler-driven tasks triggered from Timer3 (1ms tick)
  if (scheduler_ms_flag)
  {
    // clear flag and atomically snapshot ADC accumulators
    noInterrupts();
    scheduler_ms_flag = 0;
    uint16_t currentSum = 0;
    uint16_t currentCount = 0;
    // ACS712_snapshotAndClear is non-atomic by design; perform snapshot inside this interrupt-disabled section
    ACS712_snapshotAndClear(&currentSum, &currentCount);
    interrupts();

    // Apply averaged ADC value if samples were collected
    if (currentCount > 0)
    {
      int avg = (int)(currentSum / currentCount);
      ACS712_setLastADC(avg);
    }

    static uint16_t schedCounter = 0;
    schedCounter++;

    // 1) Every loop: current control trigger (set tick and perform update)
    ACS712_update();

    // 1.2) Every other loop: request FFB/USB processing
    if (schedCounter & 1)
    {
      Joystick.getUSBPID();
    }

    // 1.3) Every odd tick: alternate pedals/gears reads (offset from USBPID which runs on even ticks)
    if (schedCounter > 9)
    {
      static bool readPedals = true;
      if (readPedals)
      {
        #if PEDALS
        Pedals_update();
        #endif
      }
      else
      {
        #if GEARS
        Gears_update();
        #endif
      }
      readPedals = !readPedals;
      schedCounter = 0;
    }

    // Wheel update runs each scheduler cycle to update axis and apply FFB/motor targets
    #if WHEEL
    Wheel_update();
    #endif

    // 1.4) Debug manager at end of cycle
    DebugManager_update();
  }
}
