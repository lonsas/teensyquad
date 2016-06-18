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
    pitch_offset = 0;
    roll_offset = 0;
    yaw_offset = 0;

}

void complementary_filter::calibrateAngle(float ax, float ay, float az) {
    pitch_offset = atan2(ay, az)/3.14f;
    roll_offset = atan2(ax, az)/3.14f;

}

void complementary_filter::update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    pitch += gx*dt*250.0/32768*1.5f;
    roll += gy*dt*250.0/32768*1.5f;
    yaw += gz*dt*250.0/32768*1.5f;
    if(pitch > 1) {
        pitch -= 2;
    } else if (pitch < -1) {
        pitch += 2;
    }
    if(roll > 1) {
        roll -= 2;
    } else if (roll < -1) {
        roll += 2;
    }
    if(yaw > 1) {
        yaw -= 2;
    } else if (yaw < -1) {
        yaw += 2;
    }
    int32_t forceMagnitudeApprox = sqrt(ax*ax + ay*ay + az*az);
    if (forceMagnitudeApprox > 16600 && forceMagnitudeApprox < 17000) {
        if(fabs(roll) < 0.4f || fabs(roll) > 0.6f) {
            apitch = atan2(ay, az)/3.14f - pitch_offset;
            pitch = 0.995f*pitch + 0.005f*apitch;
        }
        if(fabs(pitch) < 0.4f || fabs(pitch) > 0.6f) {
            aroll = atan2(ax, az)/3.14f - roll_offset;
            roll = 0.995f*roll + 0.005f*aroll;
        }
    }
    //y = (1-hwc)y +hwcx
}


float complementary_filter::fabs(float a) {
    return a<0 ? -a : a;
}
