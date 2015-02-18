#ifndef _MOTORS_H
#define _MOTORS_H

#include <inttypes.h>
#include <math.h>

#define YAW_CONTROLLER_ADDRESS (0x40 << 1)
#define NUMBER_OF_MOTORS (2)
#define ROTOR_POLES (11)
#define GEARING (1.0)
#define SINE_TABLE_LENGTH (900)
#define RADIANS_TO_MOTOR_SEGMENTS ((float)(ROTOR_POLES * SINE_TABLE_LENGTH)\
  / (2.0 * M_PI) / GEARING)

enum Motors {
  MOTOR_ROLL = 0,
  MOTOR_PITCH,
};

void MotorPWMTimersInit(void);

void MotorMove(enum Motors motor, int8_t segments, uint8_t shift);
void MotorMoveToAngle(enum Motors motor, float angle);

#endif //_MOTORS_H
