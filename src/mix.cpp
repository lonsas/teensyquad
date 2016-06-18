/*
 * mix.cpp
 */
#include "mix.h"



void mix(double throttle, double *pitch, double *roll, double *yaw, double *output) {
    /*  ^
     * 0 2
     * 1 3
     */

    // Make some room for the controller to act at full throttle
    throttle *= THROTTLE_MAX;

    output[0] = output[1] = output[2] = output[3] = throttle;

    //TODO: Add saturation


    /*pitch*/
    output[0] -= *pitch/4;
    output[1] += *pitch/4;
    output[2] -= *pitch/4;
    output[3] += *pitch/4;

    /*roll*/
    output[0] += *roll/4;
    output[1] += *roll/4;
    output[2] -= *roll/4;
    output[3] -= *roll/4;

    /*yaw*/
    output[0] += *yaw/4;
    output[1] -= *yaw/4;
    output[2] -= *yaw/4;
    output[3] += *yaw/4;

    //Saturate
    if(output[0] < 0) {
        output[0] = 0;
    } else if(output[0] > 1) {
        output[0] = 1;
    }

    if(output[1] < 0) {
        output[1] = 0;
    } else if(output[1] > 1) {
        output[1] = 1;
    }

    if(output[2] < 0) {
        output[2] = 0;
    } else if(output[2] > 1) {
        output[2] = 1;
    }

    if(output[3] < 0) {
        output[3] = 0;
    } else if(output[3] > 1) {
        output[3] = 1;
    }

    *pitch = - output[0] + output[1] - output[2] + output[3];
    *roll = output[0] + output[1] - output[2] - output[3];
    *yaw = output[0] - output[1] - output[2] + output[3];
}
