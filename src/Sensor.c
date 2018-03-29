#include "Sensor.h"
#include "MPU9150_c.h"
#include "MadgwickAHRS.h"
#include "MCUConf.h"
#include "math.h"

#define OMEGA_MAX 250;
#define ANGLE_MAX 3.15;

static double m_dbRollOmega;
static double m_dbPitchOmega;
static double m_dbYawOmega;

static double m_dbRollAngle;
static double m_dbPitchAngle;
static double m_dbYawAngle;

static int16_t m_gxOffset;
static int16_t m_gyOffset;
static int16_t m_gzOffset;

static int16_t m_axOffset;
static int16_t m_ayOffset;
static int16_t m_azOffset;

static inline void GyroScale(int16_t gyro[3]);

static void SensorCalibrateZero() {
    mpu9150_getMotion6(&m_axOffset, &m_ayOffset, &m_azOffset, &m_gxOffset, &m_gyOffset, &m_gzOffset);
}

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
    m_dbRollOmega = 0;
    m_dbPitchOmega = 0;
    m_dbYawOmega = 0;
    m_dbRollAngle = 0;
    m_dbPitchAngle = 0;
    m_dbYawAngle = 0;

    mpu9150_initialize();
    SensorCalibrateZero();
    MadgwickAHRSInit();
}

void SensorUpdate() {
    int16_t acc[3];
    int16_t gyro[3];
    mpu9150_getMotion6(&acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2]);
    GyroScale(gyro);
    MadgwickAHRSupdateIMU(m_dbRollOmega, m_dbPitchOmega, m_dbYawOmega, acc[0], acc[1], acc[2]);
    MadgwickAHRSGetAngles(&m_dbRollAngle, &m_dbPitchAngle, &m_dbYawAngle);
}

bool SensorOk()
{
    return true;
}

bool SensorAngleIsLevel()
{
    return ((fabs(m_dbRollAngle) <  ANGLE_TOL)  &&
            (fabs(m_dbPitchAngle) <  ANGLE_TOL)  &&
            (fabs(m_dbYawAngle) <  ANGLE_TOL));
}

bool SensorOmegaIsZero()
{
    return ((fabs(m_dbRollOmega) < OMEGA_TOL) &&
            (fabs(m_dbPitchOmega) < OMEGA_TOL)  &&
            (fabs(m_dbYawOmega) < OMEGA_TOL));
}

static inline void GyroScale(int16_t gyro[3])
{
    m_dbRollOmega = (gyro[0] - m_gxOffset) * GYRO_SCALE;
    m_dbPitchOmega = (gyro[1] - m_gyOffset) * GYRO_SCALE;
    m_dbYawOmega = (gyro[2] - m_gzOffset) * GYRO_SCALE;
}
