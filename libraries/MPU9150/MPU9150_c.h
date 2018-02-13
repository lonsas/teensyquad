#ifndef MPU9150_C_H
#define MPU9150_C_H
#include "inttypes.h"

/* Export basic functions to C interface */
#ifdef __cplusplus
extern "C" {
#endif 
void mpu9150_initialize();
void mpu9150_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
#ifdef __cplusplus
}
#endif

#endif /* MPU9150_C_H */
