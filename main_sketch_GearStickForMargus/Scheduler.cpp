#include "Scheduler.h"
#include "Config.h"
#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t scheduler_ms_flag = 0;

void Scheduler_start(void)
{
  cli();
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3  = 0;
  // 1ms tick @16MHz with prescaler /64 -> OCR3A = 249
  OCR3A = 249;
  TCCR3B |= (1 << WGM32); // CTC mode
  TCCR3B |= (1 << CS31) | (1 << CS30); // prescaler /64
  TIMSK3 |= (1 << OCIE3A); // enable compare interrupt
  sei();
}
