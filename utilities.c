#include <inttypes.h>

int16_t S16MovingAverage256(const int16_t input, int16_t samples[256],
  uint8_t* const index, int32_t* const sum)
{
  (*index)++;
  *sum -= samples[*index];
  samples[*index] = input;
  *sum += input;
  int16_t average;
  // Reinterpret sum and return value as arrays of bytes
  uint8_t* return_byte_array = (uint8_t*)&average;
  uint8_t* sum_byte_array = (uint8_t*)sum;
  // Throwing away the 8 least significant bits of sum is equivalent to dividing
  // by 256.
  return_byte_array[0] = sum_byte_array[1];
  return_byte_array[1] = sum_byte_array[2];
  return average;
}