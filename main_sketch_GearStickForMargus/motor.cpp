#include "motor.h"
#include "Config.h"
#include "Utils.h"
#include <avr/io.h>

static int lastForce = 0; // scaled user value passed to Motor_set (pre-scaling)

void Motor_init(void)
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
    lastForce = force;
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

void Motor_set(int force) { setMotor(force); }

int Motor_getLastForce(void)
{
  return lastForce;
}

// FFB helper implementations moved from Wheel.cpp to central Motor module

int Motor_rampForceToPWM(int rawForce)
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

int Motor_computeEndpointPWM(int currentPosition)
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

void Motor_selfCenter(int wheelOutput)
{
  // Internal PID maintained in motor module
  static int lastPosition = 0;
  static long integral = 0;

  const float Kp = 0.05;     // proportional gain
  const float Ki = 0.05;   // integral gain (very small!)
  const float Kd = 0.1;      // derivative gain

  int error = 0 - wheelOutput;

  // Integral with windup guard
  integral += error;
  if(integral > 10000) integral = 10000;
  if(integral < -10000) integral = -10000;

  // Derivative
  int velocity = wheelOutput - lastPosition; // wheelOutput provides current position snapshot
  lastPosition = wheelOutput;

  // PID
  float force = (Kp * error) + (Ki * integral) - (Kd * velocity);

  // Clamp to centering limit
  int pwmForce = limitVal((int)force, -MAX_CENTERING_PWM, MAX_CENTERING_PWM);

  setMotor(-pwmForce);
}
