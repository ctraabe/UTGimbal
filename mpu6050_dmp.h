#ifndef DMP_H
#define DMP_H

#include <inttypes.h>

#include "mpu6050.h"

// Sparkfun breakout or other
#define MPU6050_SPARKFUN (0)

// Set the following to 1 or 0 to enable/disable
#define DMP_OUTPUT_ACCELEROMETER   (0)
#define DMP_OUTPUT_GYRO            (0)
#define DMP_CALIBRATED_GYRO_OUTPUT (0)
#define DMP_AUTO_CALIBRATE_GYRO    (1)

// Accessors:
float dmp_quaternion(uint8_t index);
int16_t dmp_accelerometer(uint8_t index);
int16_t dmp_gyro(uint8_t index);

// Pseudo-accessors:
float dmp_roll_angle(void);
float dmp_pitch_angle(void);
float dmp_yaw_angle(void);

// Public functions:
void MPU6050DMPInit(void);
enum MPU6050Error DMPReadFIFO(void);

#endif //DMP_H
