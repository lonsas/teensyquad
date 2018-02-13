#include "MPU9150_c.h"

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

