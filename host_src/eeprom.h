#ifndef EEPROM_H
#define EEPROM_H

#include "inttypes.h"

void eeprom_read_block(void *buf, const void *addr, uint32_t len);
void eeprom_write_block(const void *buf, void *addr, uint32_t len);

#endif /* EEPROM_H */
