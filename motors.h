#ifndef _MOTORS_H
#define _MOTORS_H

#include <inttypes.h>

void MotorPWMTimersInit(void);

void MoveMotorTo(uint8_t position);

#endif //_MOTORS_H
