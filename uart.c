#include "uart.h"

#include <avr/interrupt.h>
#include <avr/io.h>


// ============================================================================+
// Private data:

static volatile uint8_t _rx_byte = 0;


// ============================================================================+
// Public functions:

void UARTInit(uint32_t baud)
{
  uint16_t ubrr0 = (uint16_t)(F_CPU / (8 * baud) - 1);

  // Set the baud rate
  UBRR0H = (uint8_t)(ubrr0 >> 8);
  UBRR0L = (uint8_t)ubrr0;
  // Set UART Double Speed (U2X)
  UCSR0A |= (1 << U2X0);
  // Enable USART0 receiver and transmitter and .
  UCSR0B = (1 << TXEN0)  // Enable the USART0 transmitter.
      | (1 << RXEN0)     // Enable the USART0 receiver.
      | (1 << TXCIE0)    // Enable the Transmit Complete interrupt.
      | (1 << RXCIE0);   // Enable the Receive Complete interrupt.
}


// ============================================================================+
// Private functions:

// -----------------------------------------------------------------------------
// This function is called upon the "USART0 Tx complete" interrupt.
ISR(USART_TX_vect)
{
}

// -----------------------------------------------------------------------------
// This function is called upon the "USART0 Rx complete" interrupt.
ISR(USART_RX_vect)
{
  _rx_byte = UDR0;
}
