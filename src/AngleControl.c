#include "AngleControl.h"
#include "PIDConf.h"
#include "Sensor.h"

Pid m_tPidAngleRoll;
Pid m_tPidAnglePitch;
Pid m_tPidAngleYaw;

double m_dbRollAngleRef;
double m_dbPitchAngleRef;
double m_dbYawAngleRef;

void setAngleRef(double dbRollAngleRef, double dbPitchAngleRef, double dbYawAngleRef) {
    m_dbRollAngleRef = dbRollAngleRef;
    m_dbPitchAngleRef = dbPitchAngleRef;
    m_dbYawAngleRef = dbYawAngleRef;
}

void angleControlSetup()
{
    /* Setup PIDs */
    m_tPidAngleRoll = tPidSetup(g_angleRollPidParameters);
    m_tPidAnglePitch = tPidSetup(g_anglePitchPidParameters);
    m_tPidAngleYaw = tPidSetup(g_angleYawPidParameters);
}

void angleCalculateControl(double * pdbOmegaRollControl, \
                              double * pdbOmegaPitchControl, \
                              double * pdbOmegaYawControl)
{
    double dbRollAngle;
    double dbPitchAngle;
    double dbYawAngle;
    SensorGetAngle(&dbRollAngle, &dbPitchAngle, &dbYawAngle);

    *pdbOmegaRollControl = dbCalculateOutput(&m_tPidAngleRoll, \
                                              m_dbRollAngleRef, \
                                              dbRollAngle);

    *pdbOmegaPitchControl = dbCalculateOutput(&m_tPidAnglePitch, \
                                               m_dbPitchAngleRef, \
                                               dbPitchAngle);

    *pdbOmegaYawControl = dbCalculateOutput(&m_tPidAngleYaw, \
                                             m_dbYawAngleRef, \
                                             dbYawAngle);

}

void angleUpdate(double dbOmegaRollControlSat, double dbOmegaPitchControlSat, double dbOmegaYawControlSat)
{
    updateState(&m_tPidAngleRoll, dbOmegaRollControlSat);
    updateState(&m_tPidAnglePitch, dbOmegaPitchControlSat);
    updateState(&m_tPidAngleYaw, dbOmegaYawControlSat);
}
