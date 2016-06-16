#include "sensor_fusion.h"

#include "math.h"
#include "inttypes.h"
#include "stdlib.h"


complementary_filter::complementary_filter(void) {
    pitch = 0;
    roll = 0;
    yaw = 0;
    apitch = 0;
    aroll = 0;
    ayaw = 0;

}


void complementary_filter::update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    pitch += gx*dt/180.0f;
    roll += gy*dt/180.0f;
    yaw += gz*dt/180.0f;
    if(pitch > 1) {
        pitch -= 2;
    } else if (pitch < -1) {
        pitch += 2;
    }
    int32_t forceMagnitudeApprox = sqrt(ax*ax + ay*ay + az*az);
    if (forceMagnitudeApprox > 16600 && forceMagnitudeApprox < 17000) {
        if(fabs(roll) < 0.25f) {
            apitch = atan2(ay, az)/3.14f;
            pitch = 0.98f*pitch + 0.02f*apitch;
        }
        if(fabs(pitch) < 0.25f) {
            aroll = atan2(ax, az)/3.14f;
            roll = 0.98f*roll + 0.02f*aroll;
        }


    }

    //yaw = 0;


    //y = (1-hwc)y +hwcx
}
float complementary_filter::fabs(float a) {
    return a<0 ? -a : a;
}
