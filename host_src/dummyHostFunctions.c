#include "MPU9150_c.h"
#include "stdio.h"
#include "Receiver.h"
#include "inttypes.h"
#include "core_pins.h"

/* mpu9150 host */
int16_t g_ax;
int16_t g_ay;
int16_t g_az;
int16_t g_gx;
int16_t g_gy;
int16_t g_gz;

void mpu9150_initialize() {
    g_ax = 0;
    g_ay = 0;
    g_az = 0;
    g_gx = 0;
    g_gy = 0;
    g_gz = 0;
}

void mpu9150_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    *ax = g_ax;
    *ay = g_ay;
    *az = g_az;
    *gx = g_gx;
    *gy = g_gy;
    *gz = g_gz;
}


/* core stuff */
int analogWriteArray[100];

void attachInterrupt(uint8_t pin, void (*function)(void), int mode)
{
    return;
}

uint32_t micros(void)
{
    return 0;
}

void pinMode(uint8_t pin, uint8_t mode)
{
    return;
}

void analogWrite(uint8_t pin, int val)
{
    analogWriteArray[pin] = val;
}


void analogWriteFrequency(uint8_t pin, float frequency)
{
    return;
}
void analogWriteResolution(uint32_t bits)
{
    return;
}

