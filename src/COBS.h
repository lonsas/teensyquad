#ifndef COBS_H
#define COBS_H

#include <inttypes.h>
#include <stddef.h>

size_t StuffData(const uint8_t *ptr, size_t length, uint8_t *dst);
size_t UnStuffData(const uint8_t *ptr, size_t length, uint8_t *dst);

#endif
