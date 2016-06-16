#ifndef __SENSOR_FUSION_H
#define __SENSOR_FUSION_H


#include "inttypes.h"

#define X 0
#define Y 1
#define Z 2



typedef struct {
    int16_t acc[3];
    int16_t gyro[3];
/*    int16_t mag[3];*/
} SensorData;


double calculateRoll(SensorData *);

class complementary_filter {
private:
    int16_t acc[3];
    int16_t gyro[3];
    float roll;
    float pitch;
    float yaw;
    float aroll;
    float apitch;
    float ayaw;
    float fabs(float a);
public:
    complementary_filter(void);
    void begin(uint32_t dt);
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    float getRoll() {
        return roll;
    }
    float getPitch() {
        return pitch;
    }
    float getYaw() {
        return yaw;
    }
};
#endif

