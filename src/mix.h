/*
 * mix.h
 */

#ifndef SRC_MIX_H_
#define SRC_MIX_H_
#include "inttypes.h"

#define THROTTLE_MAX 0.8

void mix(double throttle, double dbBatVolt, double *pitch, double *roll, double *yaw, double *output);


#endif /* SRC_MIX_H_ */
