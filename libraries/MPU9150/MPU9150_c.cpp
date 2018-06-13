#include "MPU9150_c.h"
#include "MPU9150.h"
#include "I2Cdev.h"

MPU9150 mpu9150;

void mpu9150_initialize() {
    Wire.begin(I2C_MASTER,0x0, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400, I2C_OP_MODE_DMA);
    delay(1000);
    mpu9150.initialize();
}

void mpu9150_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    mpu9150.getAcceleration(ax, ay, az);
    mpu9150.getRotation(gx, gy, gz);
    //mpu9150.getMotion6(ax, ay, az, gx, gy, gz);
}

