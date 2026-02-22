#include "motor.h"
#include "Config.h"
#include "Wheel.h"
#include "Utils.h"
#include <avr/io.h>
#include "FixedPoint.h"

static int16_t lastPWMTarget = 0; // last PWM value sent to motor

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

void Motor_set(int16_t pwmValue)
{
    lastPWMTarget = pwmValue;
    // Clamp to PWM limits
    int16_t clamped = limitVal(pwmValue, (int16_t)-MAX_PWM, (int16_t)MAX_PWM);
    
    if (clamped > 0)
    {
        OCR1A = 0;
        OCR1B = clamped;
    }
    else if (clamped < 0)
    {
        OCR1B = 0;
        OCR1A = -clamped;
    }
    else
    {
        OCR1B = 0;
        OCR1A = 0;
    }
}

int16_t Motor_getLastPWMTarget(void)
{
  return lastPWMTarget;
}

// Motor_rampForceToPWM() removed - force mapping now handled by Wheel module
// Motor_computeEndpointPWM() removed - endpoint dampening now handled by Wheel module
// Motor_selfCenter() removed - self-centering can be implemented via Wheel target forces if needed in future
