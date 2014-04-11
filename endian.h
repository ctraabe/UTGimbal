#ifndef _ENDIAN_H
#define _ENDIAN_H

#include <inttypes.h>
#include <stdlib.h>

static inline uint8_t* BigEndianArrayFromU16(uint16_t word)
{
  // Reinterpret the word as an array of bytes.
  uint8_t *byte_array = (uint8_t *)&word;
  // Reorder the bytes.
  uint8_t *ret = (uint8_t *)malloc(sizeof(word));
  ret[0] = byte_array[1];
  ret[1] = byte_array[0];
  return ret;
}

// -----------------------------------------------------------------------------
static inline int16_t BigEndianArrayToS16(uint8_t *byte_array)
{
  int16_t ret;
  // Reinterpret the return value as an array of bytes.
  uint8_t *ret_array = (uint8_t *)&ret;
  ret_array[0] = byte_array[1];
  ret_array[1] = byte_array[0];
  return ret;
}

// -----------------------------------------------------------------------------
static inline uint16_t BigEndianArrayToU16(uint8_t *byte_array)
{
  uint16_t ret;
  // Reinterpret the return value as an array of bytes.
  uint8_t *ret_array = (uint8_t *)&ret;
  ret_array[0] = byte_array[1];
  ret_array[1] = byte_array[0];
  return ret;
}

// -----------------------------------------------------------------------------
static inline int32_t BigEndianArrayToS32(uint8_t *byte_array)
{
  int32_t ret;
  // Reinterpret the return value as an array of bytes.
  uint8_t *ret_array = (uint8_t *)&ret;
  ret_array[0] = byte_array[3];
  ret_array[1] = byte_array[2];
  ret_array[2] = byte_array[1];
  ret_array[3] = byte_array[0];
  return ret;
}

#endif //_ENDIAN_H
