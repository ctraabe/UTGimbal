#ifndef DMP_H
#define DMP_H

#include <inttypes.h>

#include "mpu6050.h"

float dmp_quaternion(uint8_t index);
int16_t dmp_accelerometer(uint8_t index);
int16_t dmp_gyro(uint8_t index);
float dmp_roll_angle(void);
float dmp_pitch_angle(void);
float dmp_yaw_angle(void);

void DMPInit(void);
enum MPU6050Error DMPReadFIFO(void);

#endif //DMP_H
