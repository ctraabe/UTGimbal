#include "mag3110.h"

#include <avr/io.h>

#include "i2c.h"


// ============================================================================+
// Public functions:

void InitMAG3110(void)
{
  struct str_MasterTx
  {
    uint8_t address;
    uint8_t data[1];
  };

  union {
    struct str_MasterTx s;
    uint8_t bytes[sizeof(struct str_MasterTx)];
  } master_tx;

  // Enable automatic resets by setting AUTO_MRST_EN bit in CTRL_REG2 
  master_tx.s.address = MAG3110_CTRL_REG2;
  master_tx.s.data[0] = _BV(MAG3110_CTRL_REG2_AUTO_MRST_EN);
  SendBytesI2C(MAG3110_ADDRESS, master_tx.bytes, 2);
  while(!I2CIsIdle()) continue;  // Wait for transmission to complete

  // Put MAG3110 in ACTIVE mode
  master_tx.s.address = MAG3110_CTRL_REG1;
  master_tx.s.data[0] = _BV(MAG3110_CTRL_REG1_AC);
  SendBytesI2C(MAG3110_ADDRESS, master_tx.bytes, 2);
  while(!I2CIsIdle()) continue;  // Wait for transmission to complete
}

// -----------------------------------------------------------------------------
void ReadMAG3110(volatile uint8_t *rx_destination_ptr)
{
  RequestFromAddress(MAG3110_ADDRESS, MAG3110_RA_OUT_X_H,
    rx_destination_ptr, sizeof(struct str_MAG3110Data));
}