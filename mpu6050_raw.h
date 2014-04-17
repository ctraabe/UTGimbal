#ifndef MPU6050_RAW_H
#define MPU6050_RAW_H

#include <inttypes.h>

#include "mpu6050.h"

struct str_MPU6050RawData
{
  int16_t x_accelerometer;
  int16_t y_accelerometer;
  int16_t z_accelerometer;
  int16_t temperature;
  int16_t x_gyro;
  int16_t y_gyro;
  int16_t z_gyro;
};

void MPU6050RawInit(void);

void MPU6050RawRead(volatile uint8_t *rx_destination_ptr);

#endif //MPU6050_RAW_H
