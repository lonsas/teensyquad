#include "sensor_fusion.h"

#include "math.h"
#include "inttypes.h"

int32_t calculateRoll(SensorData *d) {
    int32_t roll;

    roll = atan2(d->acc[X], d->acc[Y]);
    return roll;

}
