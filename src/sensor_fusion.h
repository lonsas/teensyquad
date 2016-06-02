#ifndef __SENSOR_FUSION_H
#define __SENSOR_FUSION_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "inttypes.h"

#define X 0
#define Y 1
#define Z 2



typedef struct {
    int16_t acc[3];
    int16_t gyro[3];
/*    int16_t mag[3];*/
} SensorData;


int32_t calculateRoll(SensorData *);

#ifdef __cplusplus
}
#endif

#endif

