#include "sensor_fusion.h"

#include "math.h"
#include "inttypes.h"
#include "stdlib.h"
#include "fixedptc.h"


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

void complementary_filter::calibrateAngle(double ax, double ay, double az) {
    pitch_offset = atan2(ay, az)/PI;
    roll_offset = atan2(ax, az)/PI;

}

void complementary_filter::update(double gx, double gy, double gz, double ax, double ay, double az, double dt) {
    pitch += gx*dt*250/32768.0*1.5;
    roll += gy*dt*250/32768.0*1.5;
    yaw += gz*dt*250/32768.0*1.5;
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
        if(fabs(roll) < 0.4 || fabs(roll) > 0.6) {
            apitch = atan2(ay, az)/PI - pitch_offset;
            pitch = 0.995*pitch + 0.005*apitch;
        }
        if(fabs(pitch) < 0.4 || fabs(pitch) > 0.6) {
            aroll = atan2(ax, az)/PI - roll_offset;
            aroll = -aroll; //inverted for some reason
            roll = 0.995*roll + 0.005*aroll;
        }
    }
    //y = (1-hwc)y +hwcx
}



