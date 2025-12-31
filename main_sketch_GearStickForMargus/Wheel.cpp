#include "Wheel.h"
#include "Config.h"
#include "motor.h"

#if WHEEL

volatile int currentPosition = 0;
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

// Motor actuation moved to motor.cpp (setMotor / Motor_set).

int lastPosition = 0;
long integral = 0;

// self-centering and PID control moved to Motor_selfCenter() in motor.cpp.

#endif // FFB

#if FFB
// Ramp mapping: integer quadratic mapping from raw force to PWM magnitude.
// Small raw forces map to a small fraction (soft start) while larger forces
// ramp up towards MAX_PWM. The mapping ensures that raw==0 -> pwm==0,
// and raw at max -> pwm == MAX_PWM. For any non-zero raw we ensure a
// minimum perceptible PWM of ~5% of MAX_PWM.
static int rampForceToPWM(int rawForce)
{
  int maxForce = MAX_FORCES; // expected maximum force magnitude coming from joystick
  int absForce = rawForce < 0 ? -rawForce : rawForce;

  if (absForce == 0) return 0;

  int minPWM = (MAX_PWM * 5) / 100; // 5% baseline
  if (minPWM < 1) minPWM = 1;

  // Quadratic scaling (integer-friendly): pwm = minPWM + (abs^2 * (MAX_PWM-minPWM)) / (maxForce^2)
  long numerator = (long)absForce * (long)absForce * (long)(MAX_PWM - minPWM);
  long denom = (long)maxForce * (long)maxForce;
  int scaled = (int)(numerator / (denom + 1));

  int pwm = minPWM + scaled;
  if (pwm > MAX_PWM) pwm = MAX_PWM;
  return pwm;
}

static int computeEndpointPWM(void)
{
  if (currentPosition > ENCODER_MAX_VALUE)
  {
    int dist = currentPosition - ENCODER_MAX_VALUE;
    int fullRange = ENCODER_MAX_VALUE - ENCODER_MIN_VALUE;
    long pwm = ((long)dist * (long)MAX_PWM) / ( (fullRange / ENDPOINT_BAND) + 1 );
    if (pwm < (MAX_PWM * 5) / 100) pwm = (MAX_PWM * 5) / 100; // ensure perceptible
    if (pwm > MAX_PWM) pwm = MAX_PWM;
    return (int)pwm; // positive means we need to push back negative direction in setMotor usage below
  }
  else if (currentPosition < ENCODER_MIN_VALUE)
  {
    int dist = ENCODER_MIN_VALUE - currentPosition;
    int fullRange = ENCODER_MAX_VALUE - ENCODER_MIN_VALUE;
    long pwm = ((long)dist * (long)MAX_PWM) / ( (fullRange / ENDPOINT_BAND) + 1 );
    if (pwm < (MAX_PWM * 5) / 100) pwm = (MAX_PWM * 5) / 100;
    if (pwm > MAX_PWM) pwm = MAX_PWM;
    return (int)pwm; // positive means we need to push back positive direction in setMotor usage below
  }

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
  pinMode(MOTOR_PIN_A, OUTPUT);
  pinMode(MOTOR_PIN_B, OUTPUT);
  pinMode(MOTOR_PIN_ENABLE, OUTPUT);
  digitalWrite(MOTOR_PIN_ENABLE, HIGH);
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
  int wheelOutput = limitVal(currentPosition, ENCODER_MIN_VALUE, ENCODER_MAX_VALUE);
  Joystick.setXAxis(wheelOutput);
#if FFB
  effectparams[0].springPosition = wheelOutput;
  Joystick.setEffectParams(effectparams);
  Joystick.getForce(forces);
  // First, check endpoint breach and apply corrective PWM if needed
  int endpointPWM = Motor_computeEndpointPWM(wheelOutput);
  if (endpointPWM != 0)
  {
    // If position is above max, we want to drive motor negative (back towards center)
    // ENDPOINT_BAND determines how aggressively we try to bring it back: larger
    setMotor(endpointPWM);
  }
  else
  {
    // Normal force path: get the raw force from the Joystick FFB system,
    // map it to a smoother PWM curve and apply with direction.
    int rawForce = (int)forces[0];
    int sign = (rawForce < 0) ? -1 : 1;
    int pwm = Motor_rampForceToPWM(rawForce);
    pwm = limitVal(pwm, 0, MAX_PWM);
    setMotor(- (sign * pwm));
  }
#endif
}

#endif // WHEEL
