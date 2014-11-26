#include "mpu6050.h"

#include <stdlib.h>
#include <avr/io.h>

#include "i2c.h"
#include "endian.h"
#include "timer0.h"


// =============================================================================
// Public functions:

void MPU6050Init(void)
{
/*
  // Connect the interrupt signal from MPU6050 to pin D2.
  DDRD &= ~_BV(DDD2);  // Set pin D2 (int0) to input
  EIMSK |= _BV(INT0);  // Enable the interrupt on pin D2 (int0)
  EICRA |= _BV(ISC01) | _BV(ISC00);  // Set int0 to trigger on the rising edge
*/
  uint8_t tx_buffer;

  // Reset the MPU6050
  tx_buffer = _BV(MPU6050_PWR_MGMT_1_DEVICE_RESET);
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1,
    &tx_buffer, 1);
  I2CWaitUntilCompletion();
  Timer0Delay(100);

  // Turn off MPU6050 sleep enabled bit and set clock to PLL with X gyro ref.
  tx_buffer = _BV(MPU6050_PWR_MGMT_1_CLKSEL0);
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1,
    &tx_buffer, 1);
  I2CWaitUntilCompletion();
}

// -----------------------------------------------------------------------------
enum MPU6050Error MPU6050ReadFromFIFO(volatile uint8_t *rx_destination_ptr,
  uint8_t rx_destination_length, uint8_t *remaining)
{
  // Get the number of bytes currently in the FIFO
  // TODO: Make this non-blocking
  uint8_t *rx_buffer = (uint8_t *)malloc(sizeof(uint16_t));
  if (I2CRxBytesFromRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_COUNT_H,
      rx_buffer, sizeof(uint16_t)) != I2C_ERROR_NONE)
    return MPU6050_ERROR_I2C_BUSY;
  I2CWaitUntilCompletion();

  uint16_t fifo_data_length = BigEndianArrayToU16(rx_buffer);
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
void MPU6050SetAccelerometerBias(uint8_t axis, int16_t bias)
{
  // The last bit of the 2nd byte of each accelerometer offset is reserved for
  // temperature compensation and must be preserved.
  uint8_t temperature_bit;
  I2CRxBytesFromRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XA_OFFSET_H
    + (axis * 2) + 1, &temperature_bit, 1);
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
