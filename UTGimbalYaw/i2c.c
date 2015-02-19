// Much help from reference AVR312 and corresponding source.

#include "i2c.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

#include "boolean.h"


// =============================================================================
// Private data:

enum I2CMode {
  I2C_MODE_IDLE,
  I2C_MODE_ADDRESS,
  I2C_MODE_ADDRESS_ACK,
  I2C_MODE_RX,
  I2C_MODE_RX_ACK,
  I2C_MODE_TX,
} state_;

#define I2C_ADDRESS (0x40 << 1)
#define I2C_PORT PORTA
#define I2C_PIN PINA
#define I2C_DDR DDRA
#define SDA 0
#define SCL 2

static volatile uint8_t new_incoming_data_ = 0, data_in_buffer_ = 0;

static volatile uint8_t rx_buffer_[I2C_RX_BUFFER_SIZE] = {0};
static volatile int8_t rx_buffer_head_ = 0;
// static int8_t rx_buffer_tail_ = 0;


// =============================================================================
// Private function declarations:


// =============================================================================
// Accessors


// =============================================================================
// Public Functions:

void I2CInit(void)
{
  USIPP |= _BV(USIPOS);  // Use pins A0 & A2 for SDA & SCL respectively
  I2C_PORT |= _BV(SDA) | _BV(SCL);  // Set pull-ups on I2C pins
  I2C_DDR |= _BV(SCL);  // Set SCL as output
  USISR = 0xF0;  // Clear flags and set timer to 0
  USICR = _BV(USISIE) | _BV(USIWM1) | _BV(USICS1);  // Wait for 2-wire start
  // PORTA |= _BV(PORTA4);
}

// -----------------------------------------------------------------------------
// Indicates that this device has been addressed by a remote master since the
// last time this function was called. Expect data to follow.
uint8_t I2CDataIncoming(void)
{
  uint8_t ret;
  ATOMIC_BLOCK(ATOMIC_FORCEON)
  {
    ret = new_incoming_data_;
    new_incoming_data_ = 0;
  }
  return ret;
}

// -----------------------------------------------------------------------------
// Indicates that a byte has been received since the last time this function was
// called.
uint8_t I2CDataInBuffer(void)
{
  uint8_t ret;
  ATOMIC_BLOCK(ATOMIC_FORCEON)
  {
    ret = data_in_buffer_;
    data_in_buffer_ = 0;
  }
  return ret;
}

/*
// -----------------------------------------------------------------------------
// This function returns the oldest byte from the rx_buffer. It assumes that the
// rx_buffer is never allowed to overflow, so that if the head and tail point to
// the same location in the ring buffer, then there is no new data to read. If
// the head is different from the tail, the tail is advanced and the byte at
// that location is returned.
uint8_t I2CGetByteFromBuffer(void)
{
  while (!data_in_buffer_) continue;
  rx_buffer_tail_ = (rx_buffer_tail_ + 1) & I2C_RX_BUFFER_MASK;
  if (rx_buffer_tail_ == rx_buffer_head_) data_in_buffer_ = 0;
  return rx_buffer_[rx_buffer_tail_];
}

// -----------------------------------------------------------------------------
uint8_t I2CNumBytesInBuffer(void)
{
  return (rx_buffer_head_ - rx_buffer_tail_) & I2C_RX_BUFFER_MASK;
}
*/

// -----------------------------------------------------------------------------
// Simply returns the latest received byte without affecting the tail pointer of
// the ring buffer.
uint8_t I2CPeek(void)
{
  return rx_buffer_[rx_buffer_head_];
}


// =============================================================================
// Private Functions:
// A start signal has been received.

// This function sets the USI to ignore activity on SDA and SCL until a start
// condition is detected.
static inline void I2CIgnoreUntilNextStart(void)
{
  USICR = _BV(USISIE) | _BV(USIWM1) | _BV(USICS1);
}

// -----------------------------------------------------------------------------
static inline void I2CSendAck(void)
{
  // Prepare ACK
  USIDR = 0;
  // Change SDA pin to output to send ACK (no bit setting required)
  I2C_DDR |= _BV(SDA);
  // Set the counter to overflow after 2 edges (1 cycle)
  USISR = 0x0E;
}

// -----------------------------------------------------------------------------
// I2C interrupt indicating that a start condition has been detected. Get ready
// to receive address bits from the master.
ISR(USI_START_vect)
{
  // Wait for the stat condition to finish.
  while (I2C_PIN & _BV(SCL)) continue;

  // Enable counter overflow interrupt.
  USICR |= _BV(USIOIE) | _BV(USIWM0);
  USISR = 0xF0;  // Clear flags and set counter to 0.
  state_ = I2C_MODE_ADDRESS;
}

// -----------------------------------------------------------------------------
// I2C interrupt indicating that the I2C is active and waiting for the next
// instruction.
ISR(USI_OVF_vect)
{
  uint8_t usidr_buffer_ = USIDR;
  // PORTA ^= _BV(PORTA4);
  switch (state_)
  {
    case I2C_MODE_ADDRESS:
      if ((usidr_buffer_ == 0) || ((usidr_buffer_ & 0xFE) == I2C_ADDRESS))
      {
        // PORTA ^= _BV(PORTA4);
        if (!(usidr_buffer_ & 0x01))
        {
          new_incoming_data_ = 1;
          I2CSendAck();
          state_ = I2C_MODE_ADDRESS_ACK;
          // PORTA ^= _BV(PORTA4);
        }
        else
        {
          // TODO: Implement this...
          I2CIgnoreUntilNextStart();
          state_ = I2C_MODE_IDLE;
          // state_ = I2C_MODE_TX_ACK;
        }
      }
      else
      {
        // Disable counter overflow interrupt enable start condition interrupt.
        I2CIgnoreUntilNextStart();
        state_ = I2C_MODE_IDLE;
      }
      break;
    case I2C_MODE_ADDRESS_ACK:
    case I2C_MODE_RX_ACK:
      // Change SDA pin back to input
      I2C_DDR &= ~_BV(SDA);
      state_ = I2C_MODE_RX;
      // PORTA ^= _BV(PORTA4);
      break;
    case I2C_MODE_RX:
      rx_buffer_head_ = (rx_buffer_head_ + 1) & I2C_RX_BUFFER_MASK;
      rx_buffer_[rx_buffer_head_] = usidr_buffer_;
      data_in_buffer_ = 1;
      I2CSendAck();
      state_ = I2C_MODE_RX_ACK;
      break;
    default:
      break;
  }
  USISR |= _BV(USIOIF);  // Clear the overflow interrupt flag.
}
