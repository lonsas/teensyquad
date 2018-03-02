/*
 * mix.cpp
 */
#include "mix.h"



void mixDistribute(double *roll, double *pitch, double *yaw, double *mixed) {
    /*     ^
     * CW 0 2 CC
     * CC 1 3 CW
     */

    for(int i = 0; i < 4; i++) {
        mixed[i] = 0;
    }

    /*pitch*/
    mixed[0] += *pitch/4;
    mixed[1] -= *pitch/4;
    mixed[2] += *pitch/4;
    mixed[3] -= *pitch/4;

    /*roll*/
    mixed[0] += *roll/4;
    mixed[1] += *roll/4;
    mixed[2] -= *roll/4;
    mixed[3] -= *roll/4;

    /*yaw*/
    mixed[0] += *yaw/4;
    mixed[1] -= *yaw/4;
    mixed[2] -= *yaw/4;
    mixed[3] += *yaw/4;

}

void unmix(double *roll, double *pitch, double *yaw, double *mixed) {
    *pitch = mixed[0] - mixed[1] + mixed[2] - mixed[3];
    *roll = mixed[0] + mixed[1] - mixed[2] - mixed[3];
    *yaw = mixed[0] - mixed[1] - mixed[2] + mixed[3];
}

void mixOutput(double throttle, double dbBatVolt, double *mixed, double *output) {
    
    if(throttle > THROTTLE_MAX) {
        throttle = THROTTLE_MAX;
    }
    throttle = throttle * dbBatVolt;
    /* Add throttle and saturate */
    for(int i = 0; i < 4; i++) {
        mixed[i] += throttle;
        if(mixed[i] < 0) {
            mixed[i] = 0;
        } else if(output[i] > dbBatVolt) {
            mixed[i] = dbBatVolt;
        }
        output[i] = mixed[i]/dbBatVolt;
    }
}
    

void mix(double throttle, double dbBatVolt, double *pitch, double *roll, double *yaw, double *output) {
    double mixed[4];
    mixDistribute(roll, pitch, yaw, mixed);
    mixOutput(throttle, dbBatVolt, mixed, output);
    unmix(roll, pitch, yaw, mixed);
}
