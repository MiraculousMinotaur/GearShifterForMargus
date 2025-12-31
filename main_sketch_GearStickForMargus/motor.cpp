#include "motor.h"
#include "Config.h"
#include "Utils.h"
#include <avr/io.h>

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
