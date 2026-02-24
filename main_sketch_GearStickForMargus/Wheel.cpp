#include "Wheel.h"
#include "Config.h"
#include "motor.h"
#include "ACS712Driver.h"
#include "DebugManager.h"

#if WHEEL

volatile int32_t currentPosition = 0;
volatile uint8_t lastState = 0; // Last encoder state (2 bits: [A][B])


// Fast single-channel decode optimized for being called only from INT2 (A pin)
// Only acts on changes of channel A and uses channel B state to determine direction.
// This halves resolution compared to full quadrature decoding but is much faster.
inline void tick(void)
{
// 1. Read the whole D port at once
  uint8_t currentState = PIND;
  
  // 2. Extract Phase A (Pin 2, PD1) and Phase B (Pin 1, PD3)
  // We want to format this as a 2-bit number: [A][B]
  uint8_t a = (currentState >> 2) & 1;
  uint8_t b = (currentState >> 3) & 1;
  uint8_t s = (a << 1) | b;

  // 3. Direction logic: (NewPhaseA ^ OldPhaseB)
  // This is the fastest robust way to determine CW vs CCW
  if (a ^ (lastState & 1)) {
    currentPosition++;
  } else {
    currentPosition--;
  }
  
  lastState = s;
}

// ===== AVR Hardware Interrupt Handlers for INT2 (pin 0) and INT3 (pin 1) =====
// Pin 0 (INT2) and Pin 1 (INT3) on ATmega32u4 enable direct hardware interrupt support
// This bypasses Arduino's attachInterrupt layer for lower latency and less overhead
ISR(INT2_vect)
{
  tick();
}

ISR(INT3_vect)
{
  tick();
}

#if FFB
int32_t forces[2]={0};
Gains gains[2];
EffectParams effectparams[2];

int16_t Wheel_computeTargetForce(int32_t wheelPosition)
{
  int32_t totalForce = 0;
  int16_t ffbForce = (int16_t)forces[0];
  if (ffbForce != 0)
  {
    // FFB is active: use it as target
    totalForce = -ffbForce;
  }
  else// (ffbForce == 0)
  {
    // FFB is inactive: apply self-centering force based on position
    if (wheelPosition > SELFCENTER_DEADZONE) {
      totalForce = -SELFCENTER_FORCE; // Pull back toward center
    } else if (wheelPosition < -SELFCENTER_DEADZONE) {
      totalForce = SELFCENTER_FORCE; // Pull back toward center
    } else {
      totalForce = 0; // Within deadzone, no force
    }
  }

  // This is added ON TOP of other forces so you still feel the game 
  // even while hitting the limit.
  if (wheelPosition > ENCODER_MAX_VALUE)
  {
    int32_t dist = wheelPosition - ENCODER_MAX_VALUE;
    // Simple proportional spring: F = k * x
    int32_t stopForce = (dist * (int32_t)MAX_ENDPOINT_FORCES) / ENDSTOP_WIDTH_TICKS; // Adjust '100' for stiffness
    
    // Ensure it pushes back hard enough to be felt
    if (stopForce < (MAX_ENDPOINT_FORCES / 10)) stopForce = (MAX_ENDPOINT_FORCES / 10);
    
    totalForce += stopForce; // Pushes back CCW
  }
  else if (wheelPosition < ENCODER_MIN_VALUE)
  {
    int32_t dist = ENCODER_MIN_VALUE - wheelPosition;
    int32_t stopForce = (dist * (int32_t)MAX_ENDPOINT_FORCES) / ENDSTOP_WIDTH_TICKS;
    
    if (stopForce < (MAX_ENDPOINT_FORCES / 10)) stopForce = (MAX_ENDPOINT_FORCES / 10);
    
    totalForce -= stopForce; // Pushes back CW
  }

  // --- 4. FINAL CLAMP ---
  // Ensure the combined forces don't exceed your motor's hardware limits
  return (int16_t)limitVal(totalForce, -(int32_t)MAX_FORCES, (int32_t)MAX_FORCES);
}
#endif

void Wheel_begin(void)
{
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  
  // Configure AVR hardware interrupts INT2 and INT3 for pins 0 and 1
  // INT2 (pin 0) and INT3 (pin 1) are configured for "any logical change" (CHANGE mode)
  // EICRA - External Interrupt Control Register A
  // ISC21=0, ISC20=1 -> INT2 triggers on CHANGE
  // ISC31=0, ISC30=1 -> INT3 triggers on CHANGE
  EICRA &= ~((1 << ISC21) | (1 << ISC31)); // Clear bits for '0'
  EICRA |=  ((1 << ISC20) | (1 << ISC30)); // Set bits for '1'
  
  // Enable INT2 and INT3 in the External Interrupt Mask Register
  EIMSK |= (1 << INT2) | (1 << INT3);
  
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
  cli();
  int32_t wheelValue = currentPosition;
  sei();
  int32_t wheelOutput = limitVal(wheelValue, (int32_t)ENCODER_MIN_VALUE, (int32_t)ENCODER_MAX_VALUE);
  Joystick.setXAxis((int)wheelOutput);
#if FFB
  effectparams[0].springPosition = (int)wheelOutput;
  Joystick.setEffectParams(effectparams);
  Joystick.getForce(forces);
  
  // Compute target force (respects endpoints and FFB)
  int16_t targetForce = Wheel_computeTargetForce(wheelValue);
  
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
    cli();
    tel->wheel_position = currentPosition;
    sei();
  }
}
#endif

#endif // WHEEL
