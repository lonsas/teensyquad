#ifndef MPU9150_C_H
#define MPU9150_C_H

/* Export basic functions to C interface */
#ifdef __cplusplus
extern "C" {
#endif 
void mpu9150_initialize();
void mpu9150_getMotion6();
#ifdef __cplusplus
}
#endif

#endif /* MPU9150_C_H */
