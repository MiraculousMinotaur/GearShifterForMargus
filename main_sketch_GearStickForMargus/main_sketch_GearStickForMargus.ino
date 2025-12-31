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
#endif

  // Module initializations
#if PEDALS
  Pedals_begin();
#endif

#if GEARS
  Gears_begin();
#endif

#if WHEEL
  Wheel_begin();
#endif

  // ACS712 current-sensor driver
  ACS712_begin();

  // Initialize Joystick Library
  Joystick.begin(true);

  // Start the 1ms scheduler and debug manager
#if FFB
  Scheduler_start();
#endif
  DebugManager_init();
}


#if FFB
ISR(TIMER3_COMPA_vect){
  // Minimal ISR: set scheduler flag only. Heavy work happens in main context.
  scheduler_ms_flag = 1;
}
#endif

void loop() 
{
  // Module updates are scheduled by the 1ms Scheduler (see Scheduler_start and scheduler flag handling).

  // ACS712 driver background sampling & command processing
  ACS712_backgroundTask();

  // Scheduler-driven tasks triggered from Timer3 (1ms tick)
  if (scheduler_ms_flag)
  {
    // clear flag atomically
    noInterrupts();
    scheduler_ms_flag = 0;
    interrupts();

    static uint16_t schedCounter = 0;
    schedCounter++;

    // 1) Every loop: current control trigger (set tick and perform update)
    ACS712_onTimerTick_ISR();
    ACS712_update();

    // 1.2) Every other loop: request FFB/USB processing
    if ((schedCounter & 1) == 0)
    {
      Joystick.getUSBPID();
    }

    // 1.3) Every 10th loop: alternate pedals/gears reads
    if ((schedCounter % 10) == 0)
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
    }

    // Wheel update runs each scheduler cycle to update axis and apply FFB/motor targets
    #if WHEEL
    Wheel_update();
    #endif

    // 1.4) Debug manager at end of cycle
    DebugManager_update();
  }
}
