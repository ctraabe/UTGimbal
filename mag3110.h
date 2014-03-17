#ifndef MAG3110_H
#define MAG3110_H

#include <inttypes.h>

#define MAG3110_ADDRESS             0x0E << 1

#define MAG3110_RA_DR_STATUS        0x00
#define MAG3110_RA_OUT_X_H          0x01
#define MAG3110_RA_OFF_X_H          0x09
#define MAG3110_CTRL_REG1           0x10
#define MAG3110_CTRL_REG2           0x11

#define MAG3110_CTRL_REG1_DR2          7
#define MAG3110_CTRL_REG1_DR1          6
#define MAG3110_CTRL_REG1_DR0          5
#define MAG3110_CTRL_REG1_OS1          4
#define MAG3110_CTRL_REG1_OS0          3
#define MAG3110_CTRL_REG1_AC           0

#define MAG3110_CTRL_REG2_AUTO_MRST_EN 0

struct str_MAG3110Data
{
  int16_t x_magnetometer;
  int16_t y_magnetometer;
  int16_t z_magnetometer;
};

void InitMAG3110(void);

void ReadMAG3110(volatile uint8_t *rx_destination_ptr);

#endif //MAG3110_H
