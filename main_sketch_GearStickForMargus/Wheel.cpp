#include "Wheel.h"
#include "Config.h"
#include "motor.h"
#include "ACS712Driver.h"
#include "DebugManager.h"

#if WHEEL

volatile int32_t currentPosition = 0;
bool isOutOfRange = false;

// Encoder pins on PORTD: A=pin2 (bit 2), B=pin3 (bit 3)
#define ENCODER_SHIFT 2
#define ENCODER_MASK 0x03

// Precomputed lookup table for encoder state transitions
// Index: (oldState << 2) | newState (0-15)
// Values: position delta (-1, 0, +1)
static const int8_t encoderTable[16] = {
  0,  +1, -1,  0,  // oldState=0 (00): 00->+1, 01->-1
  -1,  0,  0, +1,  // oldState=1 (01): 00->-1, 11->+1
  +1,  0,  0, -1,  // oldState=2 (10): 00->+1, 11->-1
  0,  -1, +1,  0   // oldState=3 (11): 01->-1, 10->+1
};

// Optimized encoder callback: ~20-30 cycles vs ~200 cycles
static void tick(void)
{
  static uint8_t oldState = 0;
  
  // Read both encoder pins (A=bit2, B=bit3 on PORTD) in 1 CPU cycle
  uint8_t pins = PIND;
  uint8_t newState = (pins >> ENCODER_SHIFT) & ENCODER_MASK;
  
  // Lookup table: compute index from old and new state
  uint8_t tableIdx = (oldState << 2) | newState;
  currentPosition += encoderTable[tableIdx];
  
  oldState = newState;
}

#if FFB
int32_t forces[2]={0};
Gains gains[2];
EffectParams effectparams[2];

// Timer3 scheduling is handled by the Scheduler module now.
// beginFFBRequestTimer moved to Scheduler_start() to provide a central 1ms tick.

// PWM initialization moved to Motor_init() in motor.cpp
// void initPWM() no longer defined here.

// Motor control now goes through ACS712 only, centralizing all Motor_set() calls.
// Wheel provides target force via ACS712_setTargetFromForce().

#endif // FFB

#if FFB
// Wheel computes target forces (FFB + endpoint limiting + self-centering) to send to ACS712.
// This replaces direct motor control with force target setting, allowing the
// PI controller in ACS712 to smoothly regulate motor current to achieve targets.

// Self-centering: constant force toward center when no FFB
#define SELFCENTER_FORCE 20  // constant force magnitude toward center

static int16_t Wheel_computeTargetForce(void)
{
  // Check for endpoint breach and apply corrective (dampening) force if needed
  if (currentPosition > ENCODER_MAX_VALUE)
  {
    // Over max: compute dampening force to push back (positive to return toward center)
    int32_t dist = currentPosition - ENCODER_MAX_VALUE;
    int32_t fullRange = ENCODER_MAX_VALUE - ENCODER_MIN_VALUE;
    // Map distance over limit to a positive force (pulls back toward center)
    int32_t dampingForce = (dist * (int32_t)MAX_FORCES) / ((fullRange / ENDPOINT_BAND) + 1);
    // Ensure perceptible minimum magnitude
    if (dampingForce < (MAX_FORCES / 20)) dampingForce = (MAX_FORCES / 20);
    if (dampingForce > MAX_FORCES) dampingForce = MAX_FORCES;
    return (int16_t)dampingForce;
  }
  else if (currentPosition < ENCODER_MIN_VALUE)
  {
    // Under min: compute dampening force to push back (negative to return toward center)
    int32_t dist = ENCODER_MIN_VALUE - currentPosition;
    int32_t fullRange = ENCODER_MAX_VALUE - ENCODER_MIN_VALUE;
    // Map distance over limit to a negative force (pulls back toward center)
    int32_t dampingForce = -(dist * (int32_t)MAX_FORCES) / ((fullRange / ENDPOINT_BAND) + 1);
    // Ensure perceptible minimum magnitude
    if (dampingForce > -(MAX_FORCES / 20)) dampingForce = -(MAX_FORCES / 20);
    if (dampingForce < -MAX_FORCES) dampingForce = -MAX_FORCES;
    return (int16_t)dampingForce;
  }

  // Within bounds: check for FFB input, otherwise apply self-centering
  int16_t rawForce = (int16_t)forces[0];
  if (rawForce != 0)
  {
    // FFB is active: use it as target
    return rawForce;
  }

  // FFB is inactive: apply constant self-centering force toward center
  if (currentPosition > 0)
  {
    // Pull toward center (negative)
    return -SELFCENTER_FORCE;
  }
  else if (currentPosition < 0)
  {
    // Pull toward center (positive)
    return SELFCENTER_FORCE;
  }

  // At center: no force needed
  return 0;
}
#endif

void Wheel_begin(void)
{
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A),tick,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B),tick,CHANGE);
  Joystick.setXAxisRange(ENCODER_MIN_VALUE, ENCODER_MAX_VALUE);
#if FFB
  Motor_init();
  Motor_set(0);
  // Scheduler (Timer3 1ms tick) started from main setup

  effectparams[0].springMaxPosition = ENCODER_MAX_VALUE;
  effectparams[0].springPosition = currentPosition;
  effectparams[1].springMaxPosition = 255;
  effectparams[1].springPosition = 0;

  Joystick.setGains(gains);
#endif
}

void Wheel_update(void)
{
  int32_t wheelOutput = limitVal(currentPosition, (int32_t)ENCODER_MIN_VALUE, (int32_t)ENCODER_MAX_VALUE);
  Joystick.setXAxis((int)wheelOutput);
#if FFB
  effectparams[0].springPosition = (int)wheelOutput;
  Joystick.setEffectParams(effectparams);
  Joystick.getForce(forces);
  
  // Compute target force (respects endpoints and FFB)
  int16_t targetForce = Wheel_computeTargetForce();
  
  // Send target force to ACS712 (the single motor control authority)
  // ACS712 will convert this force to current target and regulate motor via PI controller
  ACS712_setTargetFromForce(targetForce);
  
  // Auto-enable ACS712 when FFB is active (motor should move when receiving targets)
  if (!ACS712_isEnabled())
  {
    ACS712_enable(true);
  }
#endif
}

// ===== Debug Report Function =====
#if DEBUG
void Wheel_reportDebug(struct DebugTelemetry_t *tel)
{
  if (tel) {
    tel->wheel_position = currentPosition;
  }
}
#endif

#endif // WHEEL
