#include "mag3110.h"

#include <avr/io.h>

#include "i2c.h"


// =============================================================================
// Public functions:

void MAG3110Init(void)
{
  uint8_t tx_buffer[1];

  // TODO: Pin D3 is connected to the motor driver, so a different pin should be
  // used.
  // Connect the interrupt signal from MAG3110 to pin D3.
  DDRD &= ~_BV(DDD3);  // Set pin D3 (int1) to input
  EIMSK |= _BV(INT1);  // Enable the interrupt on pin D3 (int1)
  EICRA |= _BV(ISC11) | _BV(ISC10);  // Set int1 to trigger on the rising edge

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