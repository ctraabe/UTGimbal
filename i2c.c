#include "i2c.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>


// ============================================================================+
// Private data:

enum { I2C_MODE_IDLE, I2C_MODE_TX, I2C_MODE_RX, I2C_MODE_TX_THEN_RX };

volatile uint8_t _i2c_error = I2C_ERROR_NONE, _i2c_mode = I2C_MODE_IDLE;
volatile uint8_t _rx_destination_len, *_rx_destination_ptr;
volatile uint8_t _tx_source_len, *_tx_source_ptr;

uint8_t _data_address, _slave_address;


// ============================================================================+
// Private function declarations:

static void I2CStart(uint8_t i2c_mode);
static void I2CStop(void);


// ============================================================================+
// Public Functions:

// Accessors
uint8_t i2c_error(void)
{
  return _i2c_error;
}

// -----------------------------------------------------------------------------
uint8_t I2CIsIdle(void)
{
  return _i2c_mode == I2C_MODE_IDLE;
}

// -----------------------------------------------------------------------------
void InitI2C(uint32_t speed)
{
  uint8_t sreg = SREG;  // Save the global interrupt flag.
  cli();  // Disable interrupts.
  PORTC |= _BV(PORTC5) | _BV(PORTC4);  // Pull up SDA and SCL.
  TWSR &= ~(_BV(TWPS1) | _BV(TWPS0));  // Set the prescaler to 1.
  TWBR = ((F_CPU / speed) - 16) / 2;  // Set the bitrate.
  SREG = sreg;  // Restore the global interrupt flag.
}

// -----------------------------------------------------------------------------
void ResetI2C(void)
{
  I2CStop();
}

// -----------------------------------------------------------------------------
void RequestBytesI2C(uint8_t slave_address, uint8_t *rx_destination_ptr,
  uint8_t rx_destination_len)
{
  _slave_address = slave_address;
  _rx_destination_ptr = rx_destination_ptr;
  _rx_destination_len = rx_destination_len;
  _i2c_error = I2C_ERROR_NONE;
  I2CStart(I2C_MODE_RX);
}

void RequestFromAddress(uint8_t slave_address, uint8_t data_address,
    uint8_t *rx_destination_ptr, uint8_t rx_destination_len)
{
  _data_address = data_address;
  SendThenReceiveI2C(slave_address, &_data_address, 1, rx_destination_ptr,
    rx_destination_len);
}

// -----------------------------------------------------------------------------
void SendBytesI2C(uint8_t slave_address, uint8_t *tx_source_ptr,
  uint8_t tx_source_len)
{
  _slave_address = slave_address;
  _tx_source_ptr = tx_source_ptr;
  _tx_source_len = tx_source_len;
  _i2c_error = I2C_ERROR_NONE;
  I2CStart(I2C_MODE_TX);
}

// -----------------------------------------------------------------------------
void SendThenReceiveI2C(uint8_t slave_address, uint8_t *tx_source_ptr,
    uint8_t tx_source_len, uint8_t *rx_destination_ptr,
    uint8_t rx_destination_len)
{
  _slave_address = slave_address;
  _tx_source_ptr = tx_source_ptr;
  _tx_source_len = tx_source_len;
  _rx_destination_ptr = rx_destination_ptr;
  _rx_destination_len = rx_destination_len;
  _i2c_error = I2C_ERROR_NONE;
  I2CStart(I2C_MODE_TX_THEN_RX);
}


// ============================================================================+
// Private Functions:

static void I2CReadByte(void)
{
  *_rx_destination_ptr = TWDR;
  _rx_destination_ptr++;
  _rx_destination_len--;
}

// -----------------------------------------------------------------------------
// Initiate data reception and acknowledge the result.
static void I2CRxAck(void)
{
  TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE);
}

// -----------------------------------------------------------------------------
// Initiate data reception and do not acknowledge the result (don't send more).
static void I2CRxNAck(void)
{
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE);
}

// -----------------------------------------------------------------------------
// Give a start or repeated start signal.
static void I2CStart(uint8_t i2c_mode)
{
  _i2c_mode = i2c_mode;
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE);
}

// -----------------------------------------------------------------------------
// Give the stop stop signal.
static void I2CStop(void)
{
  TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
  _i2c_mode = I2C_MODE_IDLE;
}

// -----------------------------------------------------------------------------
// Initiate or continue transmission from a buffer.
static void I2CTxBuffer(void)
{
  TWDR = *_tx_source_ptr;
  _tx_source_ptr++;
  _tx_source_len--;
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE);
}

// -----------------------------------------------------------------------------
// Initiate transmission of a single byte.
static void I2CTxByte(uint8_t byte)
{
  TWDR = byte;
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE);
}

// -----------------------------------------------------------------------------
// Go to the next communication phase.
static void Next(void)
{
  //TODO: extend this for multiple reads and writes
  switch (_i2c_mode) {
    case I2C_MODE_TX_THEN_RX:
      I2CStart(I2C_MODE_RX);
      break;
    case I2C_MODE_TX:
    case I2C_MODE_RX:
    default:
      I2CStop();
      break;
  }
}

// -----------------------------------------------------------------------------
// Interrupt received during a transmit phase. Process accordingly.
static void ProcessTxInterrupt(void)
{
  switch (TWSR) {
    case TW_START:  // Start condition transmitted
    case TW_REP_START:  // Repeated start condition transmitted
      // Send a write request to the desired slave address.
      I2CTxByte(_slave_address + TW_WRITE);
      break;
    case TW_MT_SLA_ACK:  // SLA+W transmitted, ACK received
    case TW_MT_DATA_ACK:  // Data transmitted, ACK received
      if (_tx_source_len > 0) {
        I2CTxBuffer();
      } else {
        // Target is erroneously expecting more data
        _i2c_error = I2C_ERROR_ACK;
        Next();
      }
      break;
    case TW_MT_DATA_NACK:  // Data transmitted, NACK received
      if (_tx_source_len > 0) {
        // Indicates that target has canceled reception
        _i2c_error = I2C_ERROR_NACK;
      }
      // continue
    case TW_MT_SLA_NACK:  // SLA+W transmitted, NACK received
      // Suggests that the target device is not present
      _i2c_error = I2C_ERROR_NO_REPLY;
      Next();
      break;
    case TW_MT_ARB_LOST:  // arbitration lost in SLA+W or data
    case TW_NO_INFO:  // no state information available
    case TW_BUS_ERROR:  // illegal start or stop condition
    default:
      // Unexpected status message. Send stop.
      _i2c_error = I2C_ERROR_OTHER;
      I2CStop();
      break;
  }
}

// -----------------------------------------------------------------------------
// Interrupt received during a receive phase. Process accordingly.
static void ProcessRxInterrupt(void)
{
  switch (TWSR) {
    case TW_START:  // Start condition transmitted
    case TW_REP_START:  // Repeated start condition transmitted
      // Send a read request to the desired slave address.
      I2CTxByte(_slave_address + TW_READ);
      break;
    case TW_MR_DATA_ACK:  // Data received, ACK returned
      I2CReadByte();
      // continue
    case TW_MR_SLA_ACK:  // SLA+R transmitted, ACK received
      if (_rx_destination_len > 1) I2CRxAck();
      else I2CRxNAck();  // Don't send acknowledgment following last reception.
      break;
    case TW_MR_DATA_NACK:  // Data received, NACK returned
      I2CReadByte();
      Next();
      break;
    case TW_MR_SLA_NACK:  // SLA+R transmitted, NACK received
      // Suggests that the target device is not present
      _i2c_error = I2C_ERROR_NO_REPLY;
      Next();
      break;
    case TW_MR_ARB_LOST:  // Arbitration lost in SLA+R or NACK
    case TW_NO_INFO:  // No state information available
    case TW_BUS_ERROR:  // Illegal start or stop condition
    default:
      // Unexpected status message. Send stop.
      _i2c_error = I2C_ERROR_OTHER;
      I2CStop();
      break;
  }
}

// -----------------------------------------------------------------------------
// I2C interrupt indicating that the I2C is active and waiting for the next
// instruction.
ISR(TWI_vect)
{
  switch (_i2c_mode) {
    case I2C_MODE_TX:
    case I2C_MODE_TX_THEN_RX:
      ProcessTxInterrupt();
      break;
    case I2C_MODE_RX:
      ProcessRxInterrupt();
      break;
    default:
      I2CStop();  // Unexpected interrupt, reset interrupt flag;
      break;
  }
}
