#ifndef _MAIN_H
#define _MAIN_H

#include <inttypes.h>

#define I2C_SPEED 400000L
#define TIMER0_FREQUENCY 125
#define UART_BAUD 500000L

#define IDLE_LIMIT 254

enum {
  MPU6050_IDLE,
  MPU6050_READING_DATA,
  MPU6050_DATA_WAITING,
};

enum {
  MAG3110_IDLE,
  MAG3110_READING_DATA,
  MAG3110_DATA_WAITING,
};

int16_t main(void) __attribute__ ((noreturn));

#endif //_MAIN_H
