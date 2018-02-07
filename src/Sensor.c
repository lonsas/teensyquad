#include "Sensor.h"
#include "MPU9150_c.h"

static double m_dbRollOmega;
static double m_dbPitchOmega;
static double m_dbYawOmega;

void SensorGetOmega(double * pdbRollOmega, double * pdbPitchOmega, double * pdbYawOmega)
{
    *pdbRollOmega = 0;
    *pdbPitchOmega = 0;
    *pdbYawOmega = 0;
}

void SensorGetAngle(double * pdbRollAngle, double * pdbPitchAngle, double * pdbYawAngle)
{
    *pdbRollAngle = 0;
    *pdbPitchAngle = 0;
    *pdbYawAngle = 0;
}


void SensorSetup() {
    mpu9150_initialize();
}

void SensorUpdate() {
    double acc[3];
    double gyro[3];
    mpu9150_getMotion6(&acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2]);
