#ifndef SENSOR_H
#define SENSOR_H

#include "stdbool.h"

#define GYRO_MAX 250.0
#define GYRO_SCALE (GYRO_MAX/(1<<15))

void SensorGetOmega(double * pdbRollOmega, double * pdbPitchOmega, double * pdbYawOmega);

void SensorGetAngle(double * pdbRollAngle, double * pdbPitchAngle, double * pdbYawAngle);

void SensorSetup();

void SensorUpdate();

bool SensorOk();

bool SensorAngleIsLevel();

bool SensorOmegaIsZero();

#endif /* SENSOR_H */
