#ifndef _UART_H
#define _UART_H

#include <inttypes.h>
#include <avr/pgmspace.h>

#define UARTPrintf(format, ...) UARTPrintf_P(PSTR(format), ##__VA_ARGS__)

void UARTInit(uint32_t baud);
void UARTTxByte(uint8_t byte);
void UARTTxBytes(uint8_t *tx_source_ptr, uint8_t tx_source_len);
volatile uint8_t * UARTRxBuffer(void);
uint8_t UARTGetRxLabel(void);
void UARTReleaseRxBuffer(void);
uint8_t UARTGetRxSize(void);
void UARTPrintf_P(const char *format, ...);

#endif //_UART_H
