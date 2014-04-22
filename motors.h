#ifndef _MOTORS_H
#define _MOTORS_H

#include <inttypes.h>
#include <math.h>

#define ROTOR_POLES (7)
#define NUMBER_OF_MOTORS (3)
#define SINE_TABLE_LENGTH (180)
#define GEARING (0.319)
#define RADIANS_TO_MOTOR_SEGMENTS ((float)(ROTOR_POLES * SINE_TABLE_LENGTH)\
  / (2.0 * M_PI) / GEARING)

enum Motors {
  MOTOR_ROLL = 0,
  MOTOR_PITCH,
  MOTOR_YAW,
};

void MotorPWMTimersInit(void);

void MotorMove(enum Motors motor, int8_t segments);
void MotorMoveToAngle(enum Motors motor, float angle);

#endif //_MOTORS_H
