#ifndef _TIMER0_H
#define _TIMER0_H

#include <inttypes.h>

#include"boolean.h"

void InitTimer0(uint8_t frequency);
bool TickTimer0(void);

#endif //_TIMER0_H
