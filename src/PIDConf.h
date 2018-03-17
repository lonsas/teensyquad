#ifndef PIDCONF_H
#define PIDCONF_H
#include "PID.h"

/* Global variables */
extern PidParameters g_gyroRollPidParameters;
extern PidParameters g_gyroPitchPidParameters;
extern PidParameters g_gyroYawPidParameters;

extern PidParameters g_angleRollPidParameters;
extern PidParameters g_anglePitchPidParameters;
extern PidParameters g_angleYawPidParameters;

void PIDConfSetDefault();
void PIDConfLoad();
void PIDConfSave();

#endif /* PIDCONF_H */
