/*
 * mix.cpp
 */
#include "mix.h"



void mix(int32_t throttle, int32_t pitch, int32_t roll, int32_t yaw) {
    /*  ^
     * 0 2
     * 1 3
     */
    int32_t output[4];
    output[0] = output[1] = output[2] = output[3] = throttle;

    /*pitch*/
    output[0] -= pitch;
    output[1] += pitch;
    output[2] -= pitch;
    output[3] += pitch;

    /*roll*/
    output[0] += roll;
    output[1] += roll;
    output[2] -= roll;
    output[3] -= roll;

    /*yaw*/
    output[0] += yaw;
    output[1] -= yaw;
    output[2] -= yaw;
    output[3] += yaw;
}
