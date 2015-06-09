#include "uart.h"

#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/io.h>


// =============================================================================
// Private data:

#define RX_BUFFER_LENGTH (32)
#define TX_BUFFER_LENGTH (103)  // 100 chars + 2 newline chars + null terminator

volatile uint8_t rx_byte_ = 0,
  rx_length_ = 0,
  tx_source_len_ = 0,
  rx_buffer_[RX_BUFFER_LENGTH],
  *tx_source_ptr_ = 0;
static char tx_buffer_[TX_BUFFER_LENGTH];


// =============================================================================
// Public functions:

void UARTInit(uint32_t baud)
{
  uint16_t ubrr0 = (uint16_t)(F_CPU / (8 * baud) - 1);

  // Set the baud rate
  UBRR0H = (uint8_t)(ubrr0 >> 8);
  UBRR0L = (uint8_t)ubrr0;
  // Set UART Double Speed (U2X)
  UCSR0A |= _BV(U2X0);
  // Enable USART0 receiver and transmitter and .
  UCSR0B = _BV(TXEN0)  // Enable the USART0 transmitter.
      | _BV(RXEN0)     // Enable the USART0 receiver.
      | _BV(RXCIE0);   // Enable the Receive Complete interrupt.
}

// -----------------------------------------------------------------------------
// TODO: return an error code in case this tx fails.
void UARTTxByte(uint8_t byte)
{
  if (!tx_source_len_ && (UCSR0A & _BV(UDRE0))) {
    UDR0 = byte;
  }
}

// -----------------------------------------------------------------------------
// TODO: return an error code in case this tx fails.
void UARTTxBytes(uint8_t *tx_source_ptr, uint8_t tx_source_len)
{
  if (!tx_source_len_ && (UCSR0A & _BV(UDRE0))) {
    tx_source_ptr_ = tx_source_ptr;
    tx_source_len_ = tx_source_len;
    UDR0 = *tx_source_ptr;
    UCSR0B |= _BV(UDRIE0);  // Enable the USART0 data register empty interrupt.
  }
}

// -----------------------------------------------------------------------------
volatile uint8_t * UARTRxBuffer(void)
{
  return rx_buffer_;
}

// -----------------------------------------------------------------------------
uint8_t UARTGetRxLabel(void)
{
  if (rx_length_)
    return rx_buffer_[1];
  else
    return 0;
}

// -----------------------------------------------------------------------------
// Free the Rx buffer the receive new data.
void UARTReleaseRxBuffer(void)
{
  rx_length_ = 0;
}

// -----------------------------------------------------------------------------
uint8_t UARTGetRxSize(void)
{
  return rx_length_;
}


// =============================================================================
// Private functions:

// This function is called upon the "USART0 data register empty" interrupt,
// indicating that the transmitter is ready to load another byte.
ISR(USART_UDRE_vect)
{
  if (--tx_source_len_) {
    UDR0 = *(++tx_source_ptr_);
  } else {
    // This interrupt is triggered whenever the data register is empty, so must
    // be disabled after the final transmission.
    UCSR0B &= ~_BV(UDRIE0);
  }
}

// -----------------------------------------------------------------------------
// This function acts like printf, but puts the result on the UART stream. It
// also adds the end-of-line characters and checks that the character buffer is
// not exceeded. Note that this function is blocking.
void UARTPrintf_P(const char *format, ...)
{
  va_list arglist;
  va_start(arglist, format);
  int length = vsnprintf_P(tx_buffer_, 101, format, arglist);
  va_end(arglist);

  if (length < 101)
  {
    sprintf_P(&tx_buffer_[length], PSTR("\n\r"));
    length += 2;
  }
  else
  {
    sprintf_P(&tx_buffer_[80], PSTR("... MESSAGE TOO LONG\n\r"));
    length = 103;
  }

  UARTTxBytes((uint8_t *)tx_buffer_, length);
}

// -----------------------------------------------------------------------------
// This function is called upon the "USART0 Rx complete" interrupt.
ISR(USART_RX_vect)
{
  static uint16_t crc = 0;
  static uint8_t crc1 = 0, crc2 = 0, ptr = 0;

  rx_byte_ = UDR0;

  if (rx_length_ == 0) {
    // RX buffer is free.
    if (ptr == 0) {
      // Check for start condition.
      if (rx_byte_ == '#') {
        ptr = 0;
        rx_buffer_[ptr++] = rx_byte_;
        crc = rx_byte_;
      }
    } else {
      // Reception is ongoing.
      if (rx_byte_ != '\r') {
        // Add byte to buffer if space remains.
        if (ptr < RX_BUFFER_LENGTH) {
          rx_buffer_[ptr++] = rx_byte_;
          crc += rx_byte_;
        } else {
          ptr = 0;
        }
      } else {
        // Compute checksum to validate reception.
        crc -= rx_buffer_[ptr - 2];
        crc -= rx_buffer_[ptr - 1];
        crc %= 4096;
        crc1 = '=' + crc / 64;
        crc2 = '=' + crc % 64;
        if ((crc1 == rx_buffer_[ptr - 2]) && (crc2 == rx_buffer_[ptr - 1])) {
          rx_length_ = ptr + 1;
          rx_buffer_[ptr] = '\r';
        }
        ptr = 0;
      }
    }
  }
}
