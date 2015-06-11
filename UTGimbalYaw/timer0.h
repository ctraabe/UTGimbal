#ifndef _TIMER0_H
#define _TIMER0_H

#include <inttypes.h>

void Timer0Init(void);

int16_t GetTimestamp(void);
int16_t GetTimestampMillisFromNow(int16_t t);
uint8_t TimestampInPast(int16_t t);
uint16_t MillisSinceTimestamp(int16_t *last_time);
void Wait(uint16_t w);

#endif
