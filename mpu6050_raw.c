#include "mpu6050_raw.h"

#include <avr/io.h>

#include "endian.h"
#include "i2c.h"


// =============================================================================
// Public functions:

// This function initializes the MPU6050 to output raw accelerometer and gyro
// readings. It will produce an interrupt when new data is ready to be read.
void MPU6050RawInit(void)
{
  MPU6050Init();

  uint8_t tx_buffer[3];

  // Turn on the 185Hz low-pass filter for the gyro and accelerometers, then
  // set the gyro and accelerometer range (consecutive destinations).
  tx_buffer[0] = _BV(MPU6050_CONFIG_DLPF_CFG0);
  // tx_buffer[1] = 0;  // 250 deg/s
  tx_buffer[1] = _BV(MPU6050_GYRO_CONFIG_FS_SEL0);  // 500 deg/s
  // tx_buffer[1] = _BV(MPU6050_GYRO_CONFIG_FS_SEL1);  // 1000 deg/s
  // tx_buffer[2] = 0;  // 2 g
  tx_buffer[2] = _BV(MPU6050_ACCEL_CONFIG_AFS_SEL0);  // 4 g
  // tx_buffer[2] = _BV(MPU6050_ACCEL_CONFIG_AFS_SEL1);  // 8 g
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG,
    tx_buffer, 3);
  I2CWaitUntilCompletion();

  // Set the interrupt signal to latch high and clear on any read, then
  // enable the interrupt on new data ready (consecutive destinations).
  tx_buffer[0] = _BV(MPU6050_INT_PIN_CFG_LATCH_INT_EN)
    | _BV(MPU6050_INT_PIN_CFG_INT_RD_CLEAR);
  tx_buffer[1] = _BV(MPU6050_INT_ENABLE_DATA_RDY_EN);
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_PIN_CFG,
    tx_buffer, 2);
  I2CWaitUntilCompletion();
}

// -----------------------------------------------------------------------------
void MPU6050RawRead(volatile uint8_t *rx_destination_ptr)
{
  I2CRxBytesFromRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H,
    rx_destination_ptr, sizeof(struct str_MPU6050RawData));
}
