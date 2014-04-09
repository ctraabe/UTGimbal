#ifndef _I2C_H
#define _I2C_H

#include <inttypes.h>

enum I2CError {
  I2C_ERROR_NONE,
  I2C_ERROR_ACK,
  I2C_ERROR_NACK,
  I2C_ERROR_NO_REPLY,
  I2C_ERROR_OTHER,
};

// Accessors
enum I2CError i2c_error(void);

// Public Functions
void I2CInit(uint32_t speed);
void I2CReset(void);
uint8_t I2CIsIdle(void);

void I2CRxBytes(uint8_t slave_address, volatile uint8_t *rx_destination_ptr,
  uint8_t rx_destination_len);
void I2CRxBytesFromRegister(uint8_t slave_address, uint8_t register_address,
  volatile uint8_t *rx_destination_ptr, uint8_t rx_destination_len);
void I2CTxBytes(uint8_t slave_address, uint8_t *tx_source_ptr,
  uint8_t tx_source_len);
void I2CTxBytesToRegister(uint8_t slave_address, uint8_t register_address,
  uint8_t *tx_source_ptr, uint8_t tx_source_len);
void I2CTxThenRxBytes(uint8_t slave_address, uint8_t *tx_source_ptr,
  uint8_t tx_source_len, volatile uint8_t *rx_destination_ptr,
  uint8_t rx_destination_len);

void I2CWaitUntilCompletion(void);

#endif  // _I2C_H
