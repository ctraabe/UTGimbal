#ifndef _MAIN_H
#define _MAIN_H

#include <inttypes.h>

#define I2C_SPEED 400000L
#define TIMER0_FREQUENCY 125
#define UART_BAUD 500000L

enum {
  MPU6050_IDLE,
  MPU6050_NEW_DATA,
  MPU6050_READING_NEW_DATA,
};

int16_t main(void) __attribute__ ((noreturn));

#endif //_MAIN_H
