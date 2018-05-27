#include "GyroControl.h"
#include "PIDConf.h"
#include "Sensor.h"
#include <math.h>

Pid m_tPidOmegaRoll;
Pid m_tPidOmegaPitch;
Pid m_tPidOmegaYaw;

double m_dbRollOmegaRef;
double m_dbPitchOmegaRef;
double m_dbYawOmegaRef;

double m_dbOmegaDotRollControlSat;
double m_dbOmegaDotPitchControlSat;
double m_dbOmegaDotYawControlSat;

void setOmegaRef(double dbRollOmegaRef, double dbPitchOmegaRef, double dbYawOmegaRef)
{
    m_dbRollOmegaRef = dbRollOmegaRef;
    m_dbPitchOmegaRef = dbPitchOmegaRef;
    m_dbYawOmegaRef = dbYawOmegaRef;
}

void gyroControlSetup()
{
    /* Setup PIDs */
    m_tPidOmegaRoll = tPidSetup(g_gyroRollPidParameters);
    m_tPidOmegaPitch = tPidSetup(g_gyroPitchPidParameters);
    m_tPidOmegaYaw = tPidSetup(g_gyroYawPidParameters);
}

void gyroCalculateControl(double * pdbOmegaDotRollControl, \
                              double * pdbOmegaDotPitchControl, \
                              double * pdbOmegaDotYawControl)
{
    double dbRollOmega;
    double dbPitchOmega;
    double dbYawOmega;
    SensorGetOmega(&dbRollOmega, &dbPitchOmega, &dbYawOmega);

    *pdbOmegaDotRollControl = dbCalculateOutput(&m_tPidOmegaRoll, \
                                                m_dbRollOmegaRef, \
                                                dbRollOmega);

    *pdbOmegaDotPitchControl = dbCalculateOutput(&m_tPidOmegaPitch, \
                                                 m_dbPitchOmegaRef, \
                                                 dbPitchOmega);

    *pdbOmegaDotYawControl = dbCalculateOutput(&m_tPidOmegaYaw, \
                                               m_dbYawOmegaRef, \
                                               dbYawOmega);

}

void gyroUpdate(double dbOmegaDotRollControlSat, double dbOmegaDotPitchControlSat, double dbOmegaDotYawControlSat)
{
    updateState(&m_tPidOmegaRoll, dbOmegaDotRollControlSat);
    updateState(&m_tPidOmegaPitch, dbOmegaDotPitchControlSat);
    updateState(&m_tPidOmegaYaw, dbOmegaDotYawControlSat);

    m_dbOmegaDotRollControlSat = dbOmegaDotRollControlSat;
    m_dbOmegaDotPitchControlSat = dbOmegaDotPitchControlSat;
    m_dbOmegaDotYawControlSat = dbOmegaDotYawControlSat;
}

void gyroTrackingSignal(double * pdbOmegaRollSat, double * pdbOmegaPitchSat, double * pdbOmegaYawSat)
{
    double dbRollOmega;
    double dbPitchOmega;
    double dbYawOmega;

    SensorGetOmega(&dbRollOmega, &dbPitchOmega, &dbYawOmega);

    if(fabs(checkSignalDiff(&m_tPidOmegaRoll, m_dbOmegaDotRollControlSat)) > 1e-1) {
      *pdbOmegaRollSat = dbRollOmega;
    } else {
      *pdbOmegaRollSat = m_dbRollOmegaRef;
    }

    if(fabs(checkSignalDiff(&m_tPidOmegaPitch, m_dbOmegaDotPitchControlSat)) > 1e-1) {
      *pdbOmegaPitchSat = dbPitchOmega;
    } else {
      *pdbOmegaPitchSat = m_dbPitchOmegaRef;
    }

    if(fabs(checkSignalDiff(&m_tPidOmegaYaw, m_dbOmegaDotYawControlSat)) > 1e-1) {
      *pdbOmegaYawSat = dbYawOmega;
    } else {
      *pdbOmegaYawSat = m_dbYawOmegaRef;
    }
}
