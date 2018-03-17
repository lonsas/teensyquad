/* Dummy host test file */

#ifndef CORE_PINS_H
#define CORE_PINS_H

#include "inttypes.h"

#define FALLING 0
#define RISING 0
#define INPUT 0
#define OUTPUT 0

#define HIGH 1
#define LOW 0

void attachInterrupt(uint8_t pin, void (*function)(void), int mode);
uint32_t micros(void);
void pinMode(uint8_t pin, uint8_t mode);
void analogWrite(uint8_t pin, int val);
void analogWriteFrequency(uint8_t pin, float frequency);
void analogWriteResolution(uint32_t bits);
uint8_t digitalRead(uint8_t pin);


#endif

