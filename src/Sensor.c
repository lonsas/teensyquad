#include "Sensor.h"
#include "MPU9150_c.h"
#include "MadgwickAHRS.h"

#define GYRO_MAX 250.0
#define GYRO_SCALE (GYRO_MAX/(1<<15))

static double m_dbRollOmega;
static double m_dbPitchOmega;
static double m_dbYawOmega;

static double m_dbRollAngle;
static double m_dbPitchAngle;
static double m_dbYawAngle;

static inline void GyroScale(int16_t gyro[3]);

void SensorGetOmega(double * pdbRollOmega, double * pdbPitchOmega, double * pdbYawOmega)
{
    *pdbRollOmega = m_dbRollOmega;
    *pdbPitchOmega = m_dbPitchOmega;
    *pdbYawOmega = m_dbYawOmega;
}

void SensorGetAngle(double * pdbRollAngle, double * pdbPitchAngle, double * pdbYawAngle)
{
    *pdbRollAngle = m_dbRollAngle;
    *pdbPitchAngle = m_dbPitchAngle;
    *pdbYawAngle = m_dbYawAngle;
}


void SensorSetup() {
    mpu9150_initialize();
}

void SensorUpdate() {
    int16_t acc[3];
    int16_t gyro[3];
    mpu9150_getMotion6(&acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2]);
    GyroScale(gyro);
    MadgwickAHRSupdateIMU(m_dbRollOmega, m_dbPitchOmega, m_dbYawOmega, acc[0], acc[1], acc[2]);
    MadgwickAHRSGetAngles(&m_dbRollAngle, &m_dbPitchAngle, &m_dbYawAngle);
}

static inline void GyroScale(int16_t gyro[3])
{
    m_dbRollOmega = gyro[0] * GYRO_SCALE;
    m_dbPitchOmega = gyro[1] * GYRO_SCALE;
    m_dbYawOmega = gyro[2] * GYRO_SCALE;
}
