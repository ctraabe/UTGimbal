/*
* This file provides I2C functionality for the ATTiny via it's Universal Serial
* Interface (USI).
*
* Presently, only the reception as a slave is implemented.
*
* Please note that there is no protection for overflowing the RX buffer, so it
* must be checked at a rate faster than reception.
*/

#ifndef _I2C_H
#define _I2C_H

#include <inttypes.h>

// The following must be a power of 2 (<=128) for the ring buffer to work.
#define I2C_RX_BUFFER_SIZE (32)
#define I2C_RX_BUFFER_MASK (I2C_RX_BUFFER_SIZE - 1)

#if ( I2C_RX_BUFFER_SIZE & I2C_RX_BUFFER_MASK )
  #error I2C_RX_BUFFER_SIZE is not a power of 2
#endif

void I2CInit(void);

// -----------------------------------------------------------------------------
// Indicates that this device has been addressed by a remote master since the
// last time this function was called. Expect data to follow.
uint8_t I2CDataIncoming(void);

// -----------------------------------------------------------------------------
// Indicates that a byte has been received since the last time this function was
// called.
uint8_t I2CDataInBuffer(void);

// -----------------------------------------------------------------------------
// This function returns the oldest byte from the rx_buffer. It assumes that the
// rx_buffer is never allowed to overflow, so that if the head and tail point to
// the same location in the ring buffer, then there is no new data to read.
uint8_t I2CGetByteFromBuffer(void);

// -----------------------------------------------------------------------------
// Simply returns the data at the head of the ring buffer.
uint8_t I2CPeek(void);

#endif  // _I2C_H
