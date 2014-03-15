#ifndef MPU6050_H
#define MPU6050_H

#include <inttypes.h>

#define MPU6050_ADDRESS_AD0_LOW     0x68 << 1 // address pin low (GND)
#define MPU6050_ADDRESS_AD0_HIGH    0x69 << 1 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_INT_ENABLE       0x38
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_PWR_MGMT_1       0x6B

#define MPU6050_CONFIG_DLPF_CFG2    2
#define MPU6050_CONFIG_DLPF_CFG1    1
#define MPU6050_CONFIG_DLPF_CFG0    0

#define MPU6050_INT_PIN_CFG_LATCH_INT_EN 5
#define MPU6050_INT_PIN_CFG_INT_RD_CLEAR 4

#define MPU6050_INT_ENABLE_DATA_RDY_EN   0

#define MPU6050_GYRO_CONFIG_FS_SEL1      4
#define MPU6050_GYRO_CONFIG_FS_SEL0      3

#define MPU6050_PWR_MGMT_1_SLEEP         6
#define MPU6050_PWR_MGMT_1_CLKSEL2       2
#define MPU6050_PWR_MGMT_1_CLKSEL1       1
#define MPU6050_PWR_MGMT_1_CLKSEL0       0

void InitMPU6050(uint8_t mpu6050_address);

void ReadMPU6050(uint8_t mpu6050_address, uint8_t *rx_destination_ptr);

#endif //MPU6050_H
