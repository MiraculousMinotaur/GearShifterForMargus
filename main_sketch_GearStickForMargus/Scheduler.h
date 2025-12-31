#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>

extern volatile uint8_t scheduler_ms_flag;

void Scheduler_start(void);

#endif // SCHEDULER_H
