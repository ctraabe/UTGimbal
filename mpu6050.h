#ifndef MPU6050_H
#define MPU6050_H

#include <inttypes.h>

#define MPU6050_ADDRESS_AD0_LOW     0x68 << 1 // address pin low (GND)
#define MPU6050_ADDRESS_AD0_HIGH    0x69 << 1 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_INT_ENABLE       0x38
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_PWR_MGMT_1       0x6B

#define MPU6050_CONFIG_DLPF_CFG2    2
#define MPU6050_CONFIG_DLPF_CFG1    1
#define MPU6050_CONFIG_DLPF_CFG0    0

// The following correspond to a 2-bit value for full gyro scale where:
// 0 = +/-250 deg/s, 1 = +/-500 deg/s, 2 = +/-1000 deg/s, and 3 = +/-2000 deg/s
#define MPU6050_GYRO_CONFIG_FS_SEL1      4
#define MPU6050_GYRO_CONFIG_FS_SEL0      3

// The following correspond to a 2-bit value for full accelerometer scale where:
// 0 = +/-2 g, 1 = +/-4 g, 2 = +/-8 g, and 3 = +/-16 g
#define MPU6050_ACCEL_CONFIG_AFS_SEL1    4
#define MPU6050_ACCEL_CONFIG_AFS_SEL0    3

#define MPU6050_INT_PIN_CFG_LATCH_INT_EN 5
#define MPU6050_INT_PIN_CFG_INT_RD_CLEAR 4

#define MPU6050_INT_ENABLE_DATA_RDY_EN   0

#define MPU6050_PWR_MGMT_1_SLEEP         6
#define MPU6050_PWR_MGMT_1_CLKSEL2       2
#define MPU6050_PWR_MGMT_1_CLKSEL1       1
#define MPU6050_PWR_MGMT_1_CLKSEL0       0

struct str_MPU6050Data
{
  int16_t x_accelerometer;
  int16_t y_accelerometer;
  int16_t z_accelerometer;
  int16_t temperature;
  int16_t x_gyro;
  int16_t y_gyro;
  int16_t z_gyro;
};

void InitMPU6050(uint8_t mpu6050_address);

void ReadMPU6050(uint8_t mpu6050_address, volatile uint8_t *rx_destination_ptr);

#endif //MPU6050_H
