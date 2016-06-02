#include "sensor_fusion.h"

#include "math.h"
#include "inttypes.h"

double calculateRoll(SensorData *d) {
    double roll;

    roll = atan2(d->acc[X], d->acc[Y]);
    return roll;
}
