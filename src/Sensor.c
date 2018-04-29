#include "Sensor.h"
#include "MPU9150_c.h"
#include "MadgwickAHRS.h"
#include "MCUConf.h"
#include <math.h>

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

static void GyroScale(int16_t gyro[3]);

static void getMotion6_corrected(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);

static void SensorAngleUpdate(double gx, double gy, double gz, double ax, double ay, double az)
{
    const double alpha = 0.5;
    double newRollAngle;
    double newPitchAngle;
    double newYawAngle;

    /* Update with gyro */
    m_dbRollAngle += gx * SAMPLE_TIME_S;
    m_dbPitchAngle += gy * SAMPLE_TIME_S;
    m_dbYawAngle += gz * SAMPLE_TIME_S;

    if(fabs(ax) > ACCEL_TOL || fabs(az) > ACCEL_TOL) {
        newPitchAngle = atan2(ax, az);
        m_dbPitchAngle = m_dbPitchAngle * alpha + (1 - alpha) * newPitchAngle;
    }
    if(fabs(ay) > ACCEL_TOL || fabs(az) > ACCEL_TOL) {
        newRollAngle = atan2(ay, az);
        m_dbRollAngle = m_dbRollAngle * alpha + (1 - alpha) * newRollAngle;
    }
    if(fabs(ax) > ACCEL_TOL || fabs(ay) > ACCEL_TOL) {
        newYawAngle = atan2(ay, ax);
        m_dbYawAngle = m_dbYawAngle * alpha + (1 - alpha) * newYawAngle;
    }
}

static void SensorCalibrateZero()
{
    int gx;
    int gy;
    int gz;
    const int iterations = 1000;
    for(int i = 0; i < iterations; i++) {
        getMotion6_corrected(&m_axOffset, &m_ayOffset, &m_azOffset, &m_gxOffset, &m_gyOffset, &m_gzOffset);
#if 0
        m_gxOffset += gx;
        m_gyOffset += gy;
        m_gzOffset += gz;
#endif
    }
#if 0
    m_gxOffset /= iterations;
    m_gyOffset /= iterations;
    m_gzOffset /= iterations;
#endif

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
    *pdbPitchAngle = -m_dbPitchAngle;
    *pdbYawAngle = -m_dbYawAngle;
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
#if 0
    MadgwickAHRSInit();
#endif
}

void SensorUpdate() {
    int16_t acc[3];
    int16_t gyro[3];
    getMotion6_corrected(&acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2]);

    GyroScale(gyro);
#if 0
    MadgwickAHRSupdateIMU(m_dbRollOmega, m_dbPitchOmega, m_dbYawOmega, acc[0], acc[1], acc[2]);
    MadgwickAHRSGetAngles(&m_dbRollAngle, &m_dbPitchAngle, &m_dbYawAngle);
#else
    SensorAngleUpdate(m_dbRollOmega, m_dbPitchOmega, m_dbYawOmega, acc[0], acc[1], acc[2]);
#endif
}

bool SensorOk()
{
    return true;
}

bool SensorAngleIsLevel()
{
    return ((fabs(m_dbRollAngle) <  ANGLE_TOL)  &&
            (fabs(m_dbPitchAngle) <  ANGLE_TOL));
}

bool SensorOmegaIsZero()
{
    return ((fabs(m_dbRollOmega) < OMEGA_TOL) &&
            (fabs(m_dbPitchOmega) < OMEGA_TOL)  &&
            (fabs(m_dbYawOmega) < OMEGA_TOL));
}

static void GyroScale(int16_t gyro[3])
{
    const double alpha = 0.1;
    m_dbRollOmega = m_dbRollOmega * alpha + (1 - alpha) * (gyro[0] - m_gxOffset) * GYRO_SCALE;
    m_dbPitchOmega = (gyro[1] - m_gyOffset) * GYRO_SCALE;
    m_dbYawOmega = (gyro[2] - m_gzOffset) * GYRO_SCALE;
}

void getMotion6_corrected(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
    mpu9150_getMotion6(ax, ay, az, gx, gy, gz);
    *ax = -*ax;
    *ay = -*ay;

    *gx = *gx;
    *gy = *gy;
    *gz = -*gz;
}
