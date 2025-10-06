#include "Wheel.h"
#include "Config.h"

#if WHEEL

volatile int currentPosition = 0;
volatile int8_t oldState = 0;
bool isOutOfRange = false;

// Encoder callback function
static void tick(void)
{
  int8_t thisState = 0;
  thisState |=  digitalRead(ENCODER_PIN_A);
  thisState |=  digitalRead(ENCODER_PIN_B)<<1;

  switch(thisState)
  {
    case 0:
      currentPosition += (2 == oldState);
      currentPosition -= (1 == oldState);
      break;
    case 1:
      currentPosition += (0 == oldState);
      currentPosition -= (3 == oldState);
      break;
    case 2:
      currentPosition += (3 == oldState);
      currentPosition -= (0 == oldState);
      break;
    case 3:
      currentPosition += (1 == oldState);
      currentPosition -= (2 == oldState);
      break;
    default:
      break;
  }
  oldState = thisState;
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
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A),tick,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B),tick,CHANGE);
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
