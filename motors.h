#ifndef _MOTORS_H
#define _MOTORS_H

#include <inttypes.h>

enum Motors {
  MOTOR_ROLL,
  MOTOR_PITCH,
  MOTOR_YAW,
};

void MotorPWMTimersInit(void);

void MotorMoveToAngle(enum Motors motor, float angle);

#endif //_MOTORS_H
