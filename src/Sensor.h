#ifndef SENSOR_H
#define SENSOR_H

#include "stdbool.h"

#define ANGLE_MAX 3.15;

#define ACCEL_MAX 4.0
#define ACCEL_SCALE (ACCEL_MAX/(1<<15))
#define ACCEL_TOL (0.1/ACCEL_SCALE)

#define GYRO_MAX 500.0
#define GYRO_SCALE (GYRO_MAX/(1<<15))

void SensorGetOmega(double * pdbRollOmega, double * pdbPitchOmega, double * pdbYawOmega);

void SensorGetAngle(double * pdbRollAngle, double * pdbPitchAngle, double * pdbYawAngle);

void SensorSetup();

void SensorUpdate();

bool SensorOk();

bool SensorAngleIsLevel();

bool SensorOmegaIsZero();

#endif /* SENSOR_H */
