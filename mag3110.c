#include "mag3110.h"

#include <avr/io.h>

#include "i2c.h"


// =============================================================================
// Public functions:

void MAG3110Init(void)
{
  uint8_t tx_buffer[1];

  // Connect the interrupt signal from MPU6050 to pin PCINT10. Enable pin change
  // interrupts for pins PCINT14..8 (bank 1) and mask all but PCINT10.
  PCICR = _BV(PCIE1);
  PCMSK1 = _BV(PCINT10);

  // Enable automatic resets by setting AUTO_MRST_EN bit in CTRL_REG2 
  tx_buffer[0] = _BV(MAG3110_CTRL_REG2_AUTO_MRST_EN);
  I2CTxBytesToRegister(MAG3110_ADDRESS, MAG3110_CTRL_REG2, tx_buffer, 1);
  I2CWaitUntilCompletion();

  // Put MAG3110 in ACTIVE mode
  tx_buffer[0] = _BV(MAG3110_CTRL_REG1_AC);
  I2CTxBytesToRegister(MAG3110_ADDRESS, MAG3110_CTRL_REG1, tx_buffer, 1);
  I2CWaitUntilCompletion();
}

// -----------------------------------------------------------------------------
void MAG3110Read(volatile uint8_t *rx_destination_ptr)
{
  I2CRxBytesFromRegister(MAG3110_ADDRESS, MAG3110_RA_OUT_X_H,
    rx_destination_ptr, sizeof(struct str_MAG3110Data));
}