#ifndef UTILITIES_H
#define UTILITIES_H

#include <inttypes.h>

int16_t S16MovingAverage256(const int16_t input, int16_t samples[],
  uint8_t* const index, int32_t* const sum);

#endif //UTILITIES_H
