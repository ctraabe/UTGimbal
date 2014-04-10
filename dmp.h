#ifndef DMP_H
#define DMP_H

#include <inttypes.h>

#include "mpu6050.h"

float dmp_quaternion(uint8_t index);

void DMPInit(void);
enum MPU6050Error DMPReadFIFO(void);

#endif //DMP_H
