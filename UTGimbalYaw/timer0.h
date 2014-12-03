#ifndef _TIMER0_H
#define _TIMER0_H

#include <inttypes.h>

void Timer0Init(void);

uint16_t SetDelay (uint16_t t);
uint8_t CheckDelay (uint16_t t);
uint16_t GetDelay(uint16_t *last_time);

void Wait(uint16_t);

#endif
