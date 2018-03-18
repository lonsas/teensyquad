#include "COBS.h"

/* Encode data and add zero byte */
size_t StuffData(const uint8_t *ptr, size_t length, uint8_t *dst)
{
  const uint8_t *end = ptr + length;
  const uint8_t *start = dst;
  uint8_t *codePtr = dst;

  dst++;
  while(ptr < end) {
      if(*ptr == 0) {
          *codePtr = (dst - codePtr);
          codePtr = dst;
      } else {
          *dst = *ptr;
      }
      dst++;
      ptr++;
  }
  *codePtr = (dst-codePtr);
  *dst++ = 0;
  return (dst - start);
}

/* Decode data */
size_t UnStuffData(const uint8_t *ptr, size_t length, uint8_t *dst)
{
  const uint8_t * start = dst;
  const uint8_t * end = ptr + length;
  const uint8_t * nextZero = ptr + *ptr;

  ptr++;
  while(ptr < end)
  {
      if(ptr == nextZero) {
          *dst = 0;
          nextZero = ptr + *ptr;
      } else {
          *dst = *ptr;
      }
      dst++;
      ptr++;
  }
  return (dst-start);
}

