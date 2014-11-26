#include "mpu6050_raw.h"

#include <avr/io.h>

#include "endian.h"
#include "i2c.h"
#include "timer0.h"


// =============================================================================
// Private data:

#define MPU6050_FIFO_DATA_SIZE (MPU6050_FIFO_ACCELEROMETER * 3 \
  * sizeof(int16_t) + MPU6050_FIFO_GYRO * 3 * sizeof(int16_t))


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
/*
  // Push all gyro and accelerometer samples to the FIFO
  tx_buffer[0] = 0;
  if (MPU6050_FIFO_ACCELEROMETER) {
    tx_buffer[0] |= _BV(MPU6050_FIFO_ENABLE_ACCEL_EN);
  }
  if (MPU6050_FIFO_GYRO) {
    tx_buffer[0] |= _BV(MPU6050_FIFO_ENABLE_XG_EN)
      | _BV(MPU6050_FIFO_ENABLE_YG_EN) | _BV(MPU6050_FIFO_ENABLE_ZG_EN);
  }
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_ENABLE,
    tx_buffer, 1);
  I2CWaitUntilCompletion();
  Timer0Delay(50);

  // Reset the FIFO
  tx_buffer[0] = _BV(MPU6050_USER_CTRL_FIFO_RESET);
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, tx_buffer,
    1);
  I2CWaitUntilCompletion();
  Timer0Delay(50);

  // Start the FIFO
  tx_buffer[0] = _BV(MPU6050_USER_CTRL_FIFO_EN);
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, tx_buffer,
    1);
  I2CWaitUntilCompletion();
*/
}

// -----------------------------------------------------------------------------
uint8_t MPU6050RawReadFIFO(void)
// enum MPU6050Error MPU6050RawReadFIFO(void)
{
  uint8_t counter = 0, remaining = 1;
  uint8_t rx_buffer[MPU6050_FIFO_DATA_SIZE];
  enum MPU6050Error error = MPU6050_ERROR_NONE;

  // TODO: Make this non-blocking
  while (remaining) {
    error = MPU6050ReadFromFIFO(rx_buffer, MPU6050_FIFO_DATA_SIZE, &remaining);
    if (error != MPU6050_ERROR_NONE)
      return error;
    I2CWaitUntilCompletion();
    ++counter;
  }

  return counter;
  // return MPU6050_ERROR_NONE;
}

// -----------------------------------------------------------------------------
void MPU6050RawRead(volatile uint8_t *rx_destination_ptr)
{
  I2CRxBytesFromRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H,
    rx_destination_ptr, sizeof(struct str_MPU6050RawData));
}
