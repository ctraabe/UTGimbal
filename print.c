#include "print.h"

uint8_t PrintEOL(uint8_t *char_array)
{
  char_array[0] = '\r';
  char_array[1] = '\n';
  return 2;
}

// -----------------------------------------------------------------------------
uint8_t PrintSpace(uint8_t *char_array)
{
  char_array[0] = ' ';
  return 1;
}

// -----------------------------------------------------------------------------
// 8-bit unsigned integers go from 0 to 255, so require 3 characters.
uint8_t PrintU8(uint16_t value, uint8_t *char_array)
{
  char_array[0] = (uint8_t)(value / 100);
  uint8_t tmp = (uint8_t)(value - (uint16_t)char_array[0] * 100);
  char_array[0] += '0';

  char_array[1] = tmp / 10;
  tmp -= char_array[1] * 10;
  char_array[1] += '0';

  char_array[2] = tmp;
  char_array[2] += '0';

  return 3;
}

// -----------------------------------------------------------------------------
// 16-bit unsigned integers go from 0 to 65535, so require 5 characters.
uint8_t PrintU16(uint32_t value, uint8_t *char_array)
{
  char_array[0] = value / 10000;
  uint16_t tmp = (uint16_t)(value - (uint32_t)char_array[0] * 10000);
  char_array[0] += '0';

  char_array[1] = tmp / 1000;
  tmp -= char_array[1] * 1000;
  char_array[1] += '0';

  return 2 + PrintU8(tmp, char_array + 2);
}

// -----------------------------------------------------------------------------
// 32-bit unsigned integers go from 0 to 4294967295, so require 10 characters.
uint8_t PrintU32(uint32_t value, uint8_t *char_array)
{
  char_array[0] = value / 1000000000;
  value -= char_array[0] * 1000000000;
  char_array[0] += '0';

  char_array[1] = value / 100000000;
  value -= char_array[1] * 100000000;
  char_array[1] += '0';

  char_array[2] = value / 10000000;
  value -= char_array[2] * 10000000;
  char_array[2] += '0';

  char_array[3] = value / 1000000;
  value -= char_array[3] * 1000000;
  char_array[3] += '0';

  char_array[4] = value / 100000;
  value -= char_array[4] * 100000;
  char_array[4] += '0';

  return 5 + PrintU16(value, char_array + 5);
}

// -----------------------------------------------------------------------------
// 8-bit signed integers go from -128 to +127, so require 4 characters.
uint8_t PrintS8(int8_t value, uint8_t *char_array)
{
  if (value < 0) {
    char_array[0] = '-';
    // Weird typecasting required for the case that value == -128
    return 1 + PrintU8((uint16_t)(-(int16_t)value), char_array + 1);
  }
  else {
    char_array[0] = '+';
    return 1 + PrintU8((uint16_t)value, char_array + 1);
  }
}

// -----------------------------------------------------------------------------
// 16-bit signed integers go from -32768 to +32767, so require 6 characters.
uint8_t PrintS16(int16_t value, uint8_t *char_array)
{
  if (value < 0) {
    char_array[0] = '-';
    // Weird typecasting required for the case that value == -32768
    return 1 + PrintU16((uint32_t)(-(int32_t)value), char_array + 1);
  }
  else {
    char_array[0] = '+';
    return 1 + PrintU16((uint32_t)value, char_array + 1);
  }
}

// -----------------------------------------------------------------------------
// 32-bit signed integers go from -2147483648 to +2147483647, so require 11
// characters.
uint8_t PrintS32(int32_t value, uint8_t *char_array)
{
  if (value < 0) {
    char_array[0] = '-';
    // Case of value == -2147483648 ignored!
    return 1 + PrintU32((uint32_t)(-value), char_array + 1);
  }
  else {
    char_array[0] = '+';
    return 1 + PrintU32((uint32_t)value, char_array + 1);
  }
}
