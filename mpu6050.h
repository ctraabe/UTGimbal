#ifndef MPU6050_H
#define MPU6050_H

#include <inttypes.h>

#define MPU6050_X_AXIS                   (0)
#define MPU6050_Y_AXIS                   (1)
#define MPU6050_Z_AXIS                   (2)

//TODO: Move these defines to mpu6050.c
#define MPU6050_ADDRESS_AD0_LOW  (0x68 << 1) // address pin low (GND)
#define MPU6050_ADDRESS_AD0_HIGH (0x69 << 1) // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS  MPU6050_ADDRESS_AD0_LOW

#define MPU6050_RA_XA_OFFSET_H        (0x06)
#define MPU6050_RA_XG_OFFSET_H        (0x13)

#define MPU6050_RA_SMPRT_DIV          (0x19)
#define MPU6050_RA_CONFIG             (0x1A)
#define MPU6050_RA_GYRO_CONFIG        (0x1B)
#define MPU6050_RA_ACCEL_CONFIG       (0x1C)
#define MPU6050_RA_FIFO_ENABLE        (0x23)
#define MPU6050_RA_INT_PIN_CFG        (0x37)
#define MPU6050_RA_INT_ENABLE         (0x38)
#define MPU6050_RA_ACCEL_XOUT_H       (0x3B)
#define MPU6050_RA_GYRO_XOUT_H        (0x43)
#define MPU6050_RA_USER_CTRL          (0x6A)
#define MPU6050_RA_PWR_MGMT_1         (0x6B)
#define MPU6050_RA_FIFO_COUNT_H       (0x72)
#define MPU6050_RA_FIFO_R_W           (0x74)

// The following compose a 3-bit enumeration of various digital low-pass filter
// settings. See the datasheet for details.
#define MPU6050_CONFIG_DLPF_CFG2         (2)
#define MPU6050_CONFIG_DLPF_CFG1         (1)
#define MPU6050_CONFIG_DLPF_CFG0         (0)

// The following correspond to a 2-bit value for full gyro scale where:
// 0 = +/-250 deg/s, 1 = +/-500 deg/s, 2 = +/-1000 deg/s, and 3 = +/-2000 deg/s
#define MPU6050_GYRO_CONFIG_FS_SEL1      (4)
#define MPU6050_GYRO_CONFIG_FS_SEL0      (3)

// The following correspond to a 2-bit value for full accelerometer scale where:
// 0 = +/-2 g, 1 = +/-4 g, 2 = +/-8 g, and 3 = +/-16 g
#define MPU6050_ACCEL_CONFIG_AFS_SEL1    (4)
#define MPU6050_ACCEL_CONFIG_AFS_SEL0    (3)

#define MPU6050_FIFO_ENABLE_XG_EN        (6)
#define MPU6050_FIFO_ENABLE_YG_EN        (5)
#define MPU6050_FIFO_ENABLE_ZG_EN        (4)
#define MPU6050_FIFO_ENABLE_ACCEL_EN     (3)

#define MPU6050_INT_PIN_CFG_LATCH_INT_EN (5)
#define MPU6050_INT_PIN_CFG_INT_RD_CLEAR (4)

#define MPU6050_INT_ENABLE_DMP_INT_EN    (1)
#define MPU6050_INT_ENABLE_DATA_RDY_EN   (0)

#define MPU6050_USER_CTRL_DMP_EN         (7)
#define MPU6050_USER_CTRL_FIFO_EN        (6)
#define MPU6050_USER_CTRL_DMP_RESET      (3)
#define MPU6050_USER_CTRL_FIFO_RESET     (2)

#define MPU6050_PWR_MGMT_1_DEVICE_RESET  (7)
#define MPU6050_PWR_MGMT_1_SLEEP         (6)
#define MPU6050_PWR_MGMT_1_CLKSEL2       (2)
#define MPU6050_PWR_MGMT_1_CLKSEL1       (1)
#define MPU6050_PWR_MGMT_1_CLKSEL0       (0)

#define MPU6050_FIFO_LENGTH           (1024)

enum MPU6050Mode {
  MPU6050_IDLE = 0,
  MPU6050_READING_DATA,
  MPU6050_DATA_WAITING,
};

enum MPU6050Error {
  MPU6050_ERROR_NONE = 0,
  MPU6050_ERROR_FIFO_EMPTY,
  MPU6050_ERROR_FIFO_FULL,
  MPU6050_ERROR_I2C_BUSY,
};

void MPU6050Init(void);

enum MPU6050Error MPU6050ReadFromFIFO(volatile uint8_t *rx_destination_ptr,
  uint8_t rx_destination_length, uint8_t *remaining);

void MPU6050SetAccelerometerBias(uint8_t axis, int16_t bias);
void MPU6050SetGyroBias(uint8_t axis, int16_t bias);

#endif //MPU6050_H
