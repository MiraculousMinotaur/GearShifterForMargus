#include "motor.h"
#include "Config.h"
#include "Wheel.h"
#include "Utils.h"
#include <avr/io.h>
#include "FixedPoint.h"

int16_t lastPWMTarget = 0; // last PWM value sent to motor
volatile int16_t targetDuty = 0;  // Set by PID
int16_t currentDuty = 0;

void Motor_init(void)
{
  pinMode(MOTOR_PIN_A, OUTPUT);
  pinMode(MOTOR_PIN_B, OUTPUT);
  pinMode(MOTOR_PIN_ENABLE, OUTPUT);
  digitalWrite(MOTOR_PIN_ENABLE, HIGH);
  cli();
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
  ICR1 = PWM_TIMER_MAX; //~21Khz
  sei();
}

void Motor_set(int16_t pwmValue)
{
    lastPWMTarget = pwmValue;
    if (pwmValue > 0)pwmValue = CLOCKWISE_BIAS(pwmValue);
    int16_t duty = limitVal(pwmValue, -MAX_PWM, MAX_PWM);
    targetDuty = duty;
    if (currentDuty < targetDuty) {
        currentDuty += SLEW_STEP;
        if(currentDuty > targetDuty) currentDuty = targetDuty; // prevent overshoot
    } else if (currentDuty > targetDuty) {
        currentDuty -= SLEW_STEP;
        if(currentDuty < targetDuty) currentDuty = targetDuty; // prevent overshoot
    }

    // 2. Ultra-fast Bridge Logic
    if (currentDuty > 0) {
        OCR1A = 0;
        OCR1B = currentDuty;
    } else {
        OCR1B = 0;
        // Absolute value for negative currentDuty
        // On AVR, this is faster than the abs() function
        OCR1A = (currentDuty < 0) ? -currentDuty : 0; // Technically if it's not smaller then 0 it has to be 0 since we already checked for positive before
    }
}

int16_t Motor_getLastPWMTarget(void)
{
  return lastPWMTarget;
}

// Motor_rampForceToPWM() removed - force mapping now handled by Wheel module
// Motor_computeEndpointPWM() removed - endpoint dampening now handled by Wheel module
// Motor_selfCenter() removed - self-centering can be implemented via Wheel target forces if needed in future
