#include "Sensor.h"
#include "MPU9150_c.h"

#define GYRO_MAX 250.0
#define GYRO_SCALE (GYRO_MAX/(1<<15))

static double m_dbRollOmega;
static double m_dbPitchOmega;
static double m_dbYawOmega;

static double m_dbRollAngle;
static double m_dbPitchAngle;
static double m_dbYawAngle;

static inline void GyroScale(double gyro[3]);

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
    GyroScale(gyro);
    MadgwickAHRSupdateIMU(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);
    MadgwickGetAngles(&m_dbRollAngle, &m_dbPitchAngle, &m_dbYawAngle);
}

static inline void GyroScale(double gyro[3])
{
    gyro[0] *= GYRO_SCALE;
    gyro[1] *= GYRO_SCALE;
    gyro[2] *= GYRO_SCALE;
}
