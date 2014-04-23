#ifndef _TIMER0_H
#define _TIMER0_H

#include <inttypes.h>

#include "boolean.h"

void Timer0Init(void);
bool Timer0Tick(void);
void Timer0Delay(uint16_t ms);

#endif //_TIMER0_H
