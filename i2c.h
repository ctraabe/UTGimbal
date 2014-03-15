#ifndef _I2C_H
#define _I2C_H

#include <inttypes.h>

enum I2CErrors {
  I2C_ERROR_NONE,
  I2C_ERROR_ACK,
  I2C_ERROR_NACK,
  I2C_ERROR_NO_REPLY,
  I2C_ERROR_OTHER,
};

// Accessors
uint8_t i2c_error(void);

uint8_t I2CIsIdle(void);

void ResetI2C(void);
void RequestBytesI2C(uint8_t slave_address,
  volatile uint8_t *rx_destination_ptr, uint8_t rx_destination_len);
void RequestFromAddress(uint8_t slave_address, uint8_t data_address,
    volatile uint8_t *rx_destination_ptr, uint8_t rx_destination_len);
void SendBytesI2C(uint8_t slave_address, uint8_t *tx_source_ptr,
  uint8_t tx_source_len);
void SendThenReceiveI2C(uint8_t slave_address, uint8_t *tx_source_ptr,
  uint8_t tx_source_len, volatile uint8_t *rx_destination_ptr,
  uint8_t rx_destination_len);

void InitI2C(uint32_t speed);

#endif  // _I2C_H
