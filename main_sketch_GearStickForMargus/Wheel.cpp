#include "Wheel.h"
#include "Config.h"

#if WHEEL

volatile int currentPosition = 0;
bool isOutOfRange = false;

#// Encoder pins on PORTD: A=pin2 (bit 0), B=pin3 (bit 1) -- board mapping differs
#define ENCODER_SHIFT 0
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
  
  // Read both encoder pins (A=bit0, B=bit1 on PORTD) in 1 CPU cycle
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

void beginFFBRequestTimer(void)
{
  cli();
  TCCR3A = 0; //set TCCR1A 0
  TCCR3B = 0; //set TCCR1B 0
  TCNT3  = 0; //counter init
  OCR3A = 1041; // ~240 Hz
  TCCR3B |= (1 << WGM32); // CTC mode
  TCCR3B |= (1 << CS31) | (1 << CS30); // prescaler /64
  TIMSK3 |= (1 << OCIE3A); // enable compare interrupt/
  sei();
}

void initPWM(void)
{
   // Clear Timer/Counter Control Register A & B
  TCCR1A = 0;
  TCCR1B = 0;

  // Mode:14 - Fast PWM, 16-bit ICRn TOP
  TCCR1A |= (1 << WGM11) | (0 << WGM10);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);

  // No prescaling
  TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);

  TCCR1A |= (1 << COM1A1) | (0 << COM1A0);
  TCCR1A |= (1 << COM1B1) | (0 << COM1B0);
  ICR1 = 0x2FF; //~21Khz
}

void setMotor(int force)
{
    force *= 3;//scale for PWM
    if(0 < force)
    {
        OCR1A = 0;
        OCR1B = force;
    }
    else if (0 > force)
    {
        OCR1B = 0;
        OCR1A = -force;
    }
    else
    {
        OCR1B = 0;
        OCR1A = 0;
    }
}

int lastPosition = 0;
long integral = 0;

const float Kp = 0.05;     // proportional gain
const float Ki = 0.05;   // integral gain (very small!)
const float Kd = 0.1;      // derivative gain
const int CENTER_LIMIT = 120; // max PWM allowed for centering

void selfCenter(int wheelOutput)
{
  int error = 0 - wheelOutput;

  // Integral with windup guard
  integral += error;
  if(integral > 10000) integral = 10000;
  if(integral < -10000) integral = -10000;

  // Derivative
  int velocity = currentPosition - lastPosition;
  lastPosition = currentPosition;

  // PID
  float force = (Kp * error) + (Ki * integral) - (Kd * velocity);

  // Clamp
  int pwmForce = limitVal((int)force, -MAX_CENTERING_PWM, MAX_CENTERING_PWM);

  setMotor(-pwmForce);
};

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
  // Use a single Pin-Change Interrupt for the port group that holds the
  // encoder pins when available. Fall back to per-pin external interrupts
  // if the compile-time PCINT registers are not present on the target MCU.
  cli(); // disable interrupts while configuring PCINT

#if defined(PCMSK2) && defined(PCIE2)
  // Typical mapping on ATmega328P: PORTD -> PCMSK2 / PCIE2
  PCMSK2 |= (1 << PD0) | (1 << PD1); // enable PCINT for PD0 and PD1
  PCICR  |= (1 << PCIE2);            // enable pin-change interrupt for PCINT[23:16] (PORTD)
#elif defined(PCMSK0) && defined(PCIE0)
  // Alternative mapping on some AVRs: use PCMSK0 / PCIE0
  PCMSK0 |= (1 << PD0) | (1 << PD1);
  PCICR  |= (1 << PCIE0);
#else
  // No PCINT register names available; fall back to attachInterrupt()
  sei(); // re-enable interrupts before calling attachInterrupt (requires interrupts)
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), tick, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), tick, CHANGE);
  // leave here — Joystick setup happens after the interrupt configuration
  Joystick.setXAxisRange(ENCODER_MIN_VALUE, ENCODER_MAX_VALUE);
  return;
#endif

  sei();
  Joystick.setXAxisRange(ENCODER_MIN_VALUE, ENCODER_MAX_VALUE);
#if FFB
  pinMode(MOTOR_PIN_A, OUTPUT);
  pinMode(MOTOR_PIN_B, OUTPUT);
  pinMode(MOTOR_PIN_ENABLE, OUTPUT);
  digitalWrite(MOTOR_PIN_ENABLE, HIGH);
  beginFFBRequestTimer();
  initPWM();
  setMotor(0);

  effectparams[0].springMaxPosition = ENCODER_MAX_VALUE;
  effectparams[0].springPosition = currentPosition;
  effectparams[1].springMaxPosition = 255;
  effectparams[1].springPosition = 0;

  Joystick.setGains(gains);
#endif
}

// Pin-change ISR for PORTD (PCINT2_vect) — delegates to the fast tick().
ISR(PCINT2_vect)
{
  tick();
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
  int endpointPWM = computeEndpointPWM();
  if (endpointPWM != 0)
  {
    // If position is above max, we want to drive motor negative (back towards center)
    if (currentPosition > ENCODER_MAX_VALUE) setMotor(-endpointPWM);
    else setMotor(endpointPWM);
  }
  else
  {
    // Normal force path: get the raw force from the Joystick FFB system,
    // map it to a smoother PWM curve and apply with direction.
    int rawForce = (int)forces[0];
    int sign = (rawForce < 0) ? -1 : 1;
    int pwm = rampForceToPWM(rawForce);
    pwm = limitVal(pwm, 0, MAX_PWM);
    setMotor(- (sign * pwm));
  }
#endif
}

#endif // WHEEL
