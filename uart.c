#include "uart.h"

#include <avr/interrupt.h>
#include <avr/io.h>


// =============================================================================
// Private data:

static volatile uint8_t _rx_byte = 0;
static volatile uint8_t _tx_source_len = 0, *_tx_source_ptr = 0;


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
  if (!_tx_source_len && (UCSR0A & _BV(UDRE0))){
    UDR0 = byte;
  }
}

// -----------------------------------------------------------------------------
// TODO: return an error code in case this tx fails.
void UARTTxBytes(uint8_t *tx_source_ptr, uint8_t tx_source_len)
{
  if (!_tx_source_len && (UCSR0A & _BV(UDRE0))) {
    _tx_source_ptr = tx_source_ptr;
    _tx_source_len = tx_source_len;
    UDR0 = *tx_source_ptr;
    UCSR0B |= _BV(UDRIE0);  // Enable the USART0 data register empty interrupt.
  }
}


// =============================================================================
// Private functions:

// This function is called upon the "USART0 data register empty" interrupt,
// indicating that the transmitter is ready to load another byte.
ISR(USART_UDRE_vect)
{
  if (--_tx_source_len) {
    UDR0 = *(++_tx_source_ptr);
  } else {
    // This interrupt is triggered whenever the data register is empty, so must
    // be disabled after the final transmission.
    UCSR0B &= ~_BV(UDRIE0);
  }
}

// -----------------------------------------------------------------------------
// This function is called upon the "USART0 Rx complete" interrupt.
ISR(USART_RX_vect)
{
  _rx_byte = UDR0;
}
