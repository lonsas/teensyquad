/*
 * mix.cpp
 */
#include "mix.h"



void mix(double throttle, double pitch, double roll, double yaw, double *output) {
    /*  ^
     * 0 2
     * 1 3
     */
    output[0] = output[1] = output[2] = output[3] = throttle;

    //TODO: Add saturation

    /*pitch*/
    output[0] -= pitch/2;
    output[1] += pitch/2;
    output[2] -= pitch/2;
    output[3] += pitch/2;

    /*roll*/
    output[0] += roll/2;
    output[1] += roll/2;
    output[2] -= roll/2;
    output[3] -= roll/2;

    /*yaw*/
    output[0] += yaw/2;
    output[1] -= yaw/2;
    output[2] -= yaw/2;
    output[3] += yaw/2;
}
