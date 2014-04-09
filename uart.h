#ifndef _UART_H
#define _UART_H

#include <inttypes.h>

void UARTInit(uint32_t baud);
void UARTTxByte(uint8_t byte);
void UARTTxBytes(uint8_t *tx_source_ptr, uint8_t tx_source_len);

#endif //_UART_H
