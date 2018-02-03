#include "GyroControl.h"
#include "PIDConf.h"
#include "Sensor.h"

Pid m_tPidOmegaRoll;
Pid m_tPidOmegaPitch;
Pid m_tPidOmegaYaw;

double m_dbRollOmegaRef;
double m_dbPitchOmegaRef;
double m_dbYawOmegaRef;

void setOmegaRef(double dbRollOmegaRef, double dbPitchOmegaRef, double dbYawOmegaRef)
{
    m_dbRollOmegaRef = dbRollOmegaRef;
    m_dbPitchOmegaRef = dbPitchOmegaRef;
    m_dbYawOmegaRef = dbYawOmegaRef;
}

void gyroControlSetup()
{
    // Setup PIDs
    m_tPidOmegaRoll = tPidSetup(g_tGyroParameters);
    m_tPidOmegaPitch = tPidSetup(g_tGyroParameters);
    m_tPidOmegaYaw = tPidSetup(g_tGyroParameters);
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
}
