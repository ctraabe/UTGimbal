#ifndef _MAIN_H
#define _MAIN_H

#include <inttypes.h>
#include <math.h>

#define RAD_2_DEG (180.0 / M_PI)

#define I2C_SPEED (400000L)
#define TIMER0_FREQUENCY (125)
#define UART_BAUD (500000L)

enum GimbalStatus {
  GIMBAL_STATUS_RUNNING = 0,
  GIMBAL_STATUS_STARTING,
  GIMBAL_STATUS_CALIBRATION,
  GIMBAL_STATUS_ERROR,
};

enum MAG3110Mode {
  MAG3110_IDLE,
  MAG3110_READING_DATA,
  MAG3110_DATA_WAITING,
};

int16_t main(void) __attribute__ ((noreturn));

#endif //_MAIN_H
