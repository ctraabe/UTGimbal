#ifndef _MOTORS_H
#define _MOTORS_H

#include <inttypes.h>
#include <math.h>

#define YAW_CONTROLLER_ADDRESS (0x40 << 1)
#define ROTOR_POLES (7)
#define GEARING (1.0)
#define SINE_TABLE_LENGTH (900)
#define RADIANS_TO_MOTOR_SEGMENTS ((float)(ROTOR_POLES * SINE_TABLE_LENGTH)\
  / (2.0 * M_PI) / GEARING)

void MotorPWMTimerInit(void);

void MotorMove(int8_t segments, uint8_t shift);
void MotorMoveToAngle(float angle);
int8_t magnetic_field_rotations(void);

#endif //_MOTORS_H
