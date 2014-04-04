#ifndef _TIMER0_H
#define _TIMER0_H

#include <inttypes.h>

#include"boolean.h"

void Timer0Init(uint8_t frequency);
bool Timer0Tick(void);

#endif //_TIMER0_H
