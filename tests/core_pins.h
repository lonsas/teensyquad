/* Dummy host test file */

#ifndef CORE_PINS_H
#define CORE_PINS_H

#include "inttypes.h"

#define FALLING 0
#define RISING 0
#define INPUT 0
#define OUTPUT 0


static void attachInterrupt(uint8_t pin, void (*function)(void), int mode)
{
    return;
}

static uint32_t micros(void)
{
    return 0;
}

static void pinMode(uint8_t pin, uint8_t mode)
{
    return;
}

static void analogWrite(uint8_t pin, int val)
{
    return;
}


static void analogWriteFrequency(uint8_t pin, float frequency)
{
    return;
}
static void analogWriteResolution(uint32_t bits)
{
    return;
}

#endif

