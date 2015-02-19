#ifndef DMP_H
#define DMP_H

#include <inttypes.h>

#include "mpu6050.h"

#define DMP_SAMPLE_RATE (200)
#define DMP_SAMPLE_TIME (1.0 / (float)DMP_SAMPLE_RATE)
#define DMP_GYRO_TO_RADPS (2000. / 32768. * 3.141592654 / 180.)

// Sparkfun breakout or other
#define MPU6050_BASECAM (0)
#define MPU6050_BLUE (1)
#define MPU6050_SPARKFUN (0)

// Set the following to 1 or 0 to enable/disable
#define DMP_OUTPUT_ACCELEROMETER   (1)
#define DMP_OUTPUT_GYRO            (1)
#define DMP_CALIBRATED_GYRO_OUTPUT (0)
#define DMP_AUTO_CALIBRATE_GYRO    (1)

enum DMPCalibrationMode {
  DMP_CALIBRATE_DONE = 0,
  DMP_CALIBRATE_START,
  DMP_CALIBRATE_GYRO_X,
  DMP_CALIBRATE_GYRO_Y,
  DMP_CALIBRATE_GYRO_Z,
  DMP_CALIBRATE_ACC_X,
  DMP_CALIBRATE_ACC_Y,
  DMP_CALIBRATE_ACC_Z,
};

// Accessors:
float dmp_quaternion(uint8_t index);
int16_t dmp_accelerometer(uint8_t index);
int16_t dmp_gyro(uint8_t index);

// Pseudo-accessors:
float dmp_roll_angle(void);  // rad
float dmp_pitch_angle(void);  // rad
float dmp_yaw_angle(void);  // rad

// Public functions:
void MPU6050DMPInit(void);
enum MPU6050Error DMPReadFIFO(void);
void DMPCalibrate(enum DMPCalibrationMode* mode, int16_t* offset, int16_t *sample);

#endif //DMP_H
