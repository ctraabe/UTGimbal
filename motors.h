#ifndef _MOTORS_H
#define _MOTORS_H

#include <inttypes.h>
#include <math.h>

#define SINE_TABLE_LENGTH (900)

#define NUMBER_OF_MOTORS (2)
#define ROTOR_POLES (11)
#define GEARING (1.0)
#define RADIANS_TO_MOTOR_SEGMENTS ((float)(ROTOR_POLES * SINE_TABLE_LENGTH)\
  / (2.0 * M_PI) / GEARING)
#define MOTOR_SEGMENT_LIMIT (400)

#define YAW_CONTROLLER_ADDRESS (0x40 << 1)
#define YAW_ROTOR_POLES (7)
#define YAW_GEARING (1.0)
#define YAW_RADIANS_TO_MOTOR_SEGMENTS ((float)(YAW_ROTOR_POLES\
  * SINE_TABLE_LENGTH) / (2.0 * M_PI) / YAW_GEARING)
#define YAW_MOTOR_SEGMENT_LIMIT (127)

enum Motors {
  MOTOR_YAW = 0,
  MOTOR_A,
  MOTOR_B
};

void MotorPWMTimersInit(void);

int16_t MotorSetAngle(enum Motors motor, float angle);
uint8_t MotorsStartup(void);
void MotorsKill(void);

#endif //_MOTORS_H
