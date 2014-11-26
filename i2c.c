#include "i2c.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>

#include "boolean.h"


// =============================================================================
// Private data:

enum I2CMode {
  I2C_MODE_IDLE,
  I2C_MODE_TX,
  I2C_MODE_RX,
  I2C_MODE_TX_THEN_RX
};

static volatile enum I2CError _i2c_error = I2C_ERROR_NONE;
static volatile enum I2CMode _i2c_mode = I2C_MODE_IDLE;
static volatile uint8_t _rx_destination_len = 0, *_rx_destination_ptr = 0;
static volatile uint8_t _tx_source_len = 0, *_tx_source_ptr = 0;
static volatile uint8_t _i2c_inactivity_counter = 0;
static volatile bool _register_address_specified = FALSE;

static uint8_t _register_address = 0x00, _slave_address = 0x00;
static i2c_callback _callback_ptr = 0;


// =============================================================================
// Private function declarations:

static void I2CStart(enum I2CMode i2c_mode);
static void I2CStop(void);


// =============================================================================
// Accessors

enum I2CError i2c_error(void)
{
  return _i2c_error;
}


// =============================================================================
// Public Functions:

void I2CInit(uint32_t speed)
{
  uint8_t sreg = SREG;  // Save the global interrupt flag.
  cli();  // Disable interrupts.
  PORTC |= _BV(PORTC5) | _BV(PORTC4);  // Pull up SDA and SCL.
  TWSR &= ~(_BV(TWPS1) | _BV(TWPS0));  // Set the prescaler to 1.
  TWBR = ((F_CPU / speed) - 16) / 2;  // Set the bitrate.
  SREG = sreg;  // Restore the global interrupt flag.
}

// -----------------------------------------------------------------------------
void I2CReset(void)
{
  I2CStop();
}

// -----------------------------------------------------------------------------
uint8_t I2CIsIdle(void)
{
  return _i2c_mode == I2C_MODE_IDLE;
}

// -----------------------------------------------------------------------------
uint8_t I2CInactivtyCounter(void)
{
  if (_i2c_mode != I2C_MODE_IDLE)
    ++_i2c_inactivity_counter;
  return _i2c_inactivity_counter;
}

// -----------------------------------------------------------------------------
enum I2CError I2CRxBytes(uint8_t slave_address,
  volatile uint8_t *rx_destination_ptr, uint8_t rx_destination_len)
{
  return I2CRxBytesCallback(slave_address, rx_destination_ptr,
    rx_destination_len, 0);
}

// -----------------------------------------------------------------------------
enum I2CError I2CRxBytesCallback(uint8_t slave_address,
  volatile uint8_t *rx_destination_ptr, uint8_t rx_destination_len,
  i2c_callback callback_ptr)
{
  if (_i2c_mode != I2C_MODE_IDLE)
    return I2C_ERROR_BUSY;
  _slave_address = slave_address;
  _rx_destination_ptr = rx_destination_ptr;
  _rx_destination_len = rx_destination_len;
  _i2c_error = I2C_ERROR_NONE;
  _callback_ptr = callback_ptr;
  I2CStart(I2C_MODE_RX);
  return I2C_ERROR_NONE;
}

// -----------------------------------------------------------------------------
enum I2CError I2CRxBytesFromRegister(uint8_t slave_address,
  uint8_t register_address, volatile uint8_t *rx_destination_ptr,
  uint8_t rx_destination_len)
{
  _register_address = register_address;
  _register_address_specified = TRUE;
  return I2CTxThenRxBytes(slave_address, 0, 0, rx_destination_ptr,
    rx_destination_len);
}

// -----------------------------------------------------------------------------
enum I2CError I2CTxBytes(uint8_t slave_address, uint8_t *tx_source_ptr,
  uint8_t tx_source_len)
{
  if (_i2c_mode != I2C_MODE_IDLE)
    return I2C_ERROR_BUSY;
  _slave_address = slave_address;
  _tx_source_ptr = tx_source_ptr;
  _tx_source_len = tx_source_len;
  _i2c_error = I2C_ERROR_NONE;
  I2CStart(I2C_MODE_TX);
  return I2C_ERROR_NONE;
}

// -----------------------------------------------------------------------------
enum I2CError I2CTxBytesToRegister(uint8_t slave_address,
  uint8_t register_address, uint8_t *tx_source_ptr, uint8_t tx_source_len)
{
  _register_address = register_address;
  _register_address_specified = TRUE;
  return I2CTxBytes(slave_address, tx_source_ptr, tx_source_len);
}

// -----------------------------------------------------------------------------
enum I2CError I2CTxThenRxBytes(uint8_t slave_address, uint8_t *tx_source_ptr,
  uint8_t tx_source_len, volatile uint8_t *rx_destination_ptr,
  uint8_t rx_destination_len)
{
  if (_i2c_mode != I2C_MODE_IDLE)
    return I2C_ERROR_BUSY;
  _slave_address = slave_address;
  _tx_source_ptr = tx_source_ptr;
  _tx_source_len = tx_source_len;
  _rx_destination_ptr = rx_destination_ptr;
  _rx_destination_len = rx_destination_len;
  _i2c_error = I2C_ERROR_NONE;
  I2CStart(I2C_MODE_TX_THEN_RX);
  return I2C_ERROR_NONE;
}

// -----------------------------------------------------------------------------
void I2CWaitUntilCompletion(void)
{
  uint16_t counter = 10000;
  while (_i2c_mode != I2C_MODE_IDLE && counter) --counter;
  if (!counter) I2CReset();
}


// =============================================================================
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
static void I2CStart(enum I2CMode i2c_mode)
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
  _i2c_inactivity_counter = 0;
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
      if (_callback_ptr) (*_callback_ptr)();
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
      if (_register_address_specified) {
        I2CTxByte(_register_address);
        _register_address_specified = FALSE;
        break;
      }
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
