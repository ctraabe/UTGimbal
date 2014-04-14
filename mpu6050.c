#include "mpu6050.h"

#include <stdlib.h>
#include <avr/io.h>

#include "endian.h"
#include "i2c.h"


// =============================================================================
// Private data:

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


// =============================================================================
// Public functions:

void MPU6050Init(void)
{
  uint8_t tx_buffer[3];

  // Turn off MPU6050 sleep enabled bit and set clock to PLL with X gyro ref.
  tx_buffer[0] = _BV(MPU6050_PWR_MGMT_1_CLKSEL0);
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1,
    tx_buffer, 1);
  I2CWaitUntilCompletion();

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
enum MPU6050Error MPU6050ReadFromFIFO(volatile uint8_t *rx_destination_ptr,
  uint8_t rx_destination_length, uint8_t *remaining)
{
  uint16_t fifo_data_length;

  // Get the number of bytes currently in the FIFO
  // TODO: Make this non-blocking
  uint8_t *rx_buffer = (uint8_t *)malloc(sizeof(uint16_t));
  I2CRxBytesFromRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_COUNT_H,
    rx_buffer, sizeof(uint16_t));
  I2CWaitUntilCompletion();
  fifo_data_length = BigEndianArrayToU16(rx_buffer);
  free(rx_buffer);

  if (fifo_data_length < rx_destination_length) {
    *remaining = 0;
    return MPU6050_ERROR_FIFO_EMPTY;
  }
  // TODO: check for FIFO overflow
  // return MPU6050_ERROR_FIFO_FULL;

  I2CRxBytesFromRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_R_W,
    rx_destination_ptr, rx_destination_length);

  *remaining = fifo_data_length / rx_destination_length - 1;
  return MPU6050_ERROR_NONE;
}

// -----------------------------------------------------------------------------
void MPU6050ReadRaw(volatile uint8_t *rx_destination_ptr)
{
  I2CRxBytesFromRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H,
    rx_destination_ptr, sizeof(struct str_MPU6050Data));
}

// -----------------------------------------------------------------------------
void MPU6050SetAccelerometerBias(uint8_t axis, int16_t bias)
{
  // The last bit of the 2nd byte of each accelerometer offset is reserved for
  // temperature compensation and must be preserved.
  uint8_t temperature_bit;
  I2CRxBytesFromRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XA_OFFSET_H
    + (axis << 1) + 1, &temperature_bit, 1);
  I2CWaitUntilCompletion();

  // Prepare the tx buffer (multiplying by 2 makes a zero in the LSB).
  uint8_t *tx_buffer = BigEndianArrayFromS16(bias * 2);
  tx_buffer[1] |= temperature_bit & 0x01;

  // Send the tx buffer.
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XA_OFFSET_H
    + (axis * 2), tx_buffer, 2);
  I2CWaitUntilCompletion();
  free(tx_buffer);
}

// -----------------------------------------------------------------------------
void MPU6050SetGyroBias(uint8_t axis, int16_t bias)
{
  uint8_t *tx_buffer = BigEndianArrayFromS16(bias);
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XG_OFFSET_H
    + (axis * 2), tx_buffer, 2);
  I2CWaitUntilCompletion();
  free(tx_buffer);
}
