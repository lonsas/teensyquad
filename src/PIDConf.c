#ifndef PIDCONF_H
#define PIDCONF_H
#include "PID.h"
#include "MCUConf.h"
#include "eeprom.h"


static const PidParameters gyroDefaultPidParameters = {
  .K = 2,
  .Ti = 0.1,
  .Td = 0,
  .Tt = 0.05,
  .b = 1.0,
  .h = 0.001,
};

static const PidParameters gyroYawDefaultPidParameters = {
  .K = 8,
  .Ti = 0.1,
  .Td = 0,
  .Tt = 0.05,
  .b = 1.0,
  .h = 0.001,
};

/* TODO: A bit unstable in angle mode with disturbances */
static const PidParameters angleDefaultPidParameters = {
  .K = 5,
  .Ti = 1,
  .Td = 0,
  .Tt = 0.2,
  .b = 1,
  .h = 0.001,
  .N = 10,
};


/* Global PID parameters */
PidParameters g_gyroRollPidParameters;
PidParameters g_gyroPitchPidParameters;
PidParameters g_gyroYawPidParameters;

PidParameters g_angleRollPidParameters;
PidParameters g_anglePitchPidParameters;
PidParameters g_angleYawPidParameters;

static PidParameters EEPROMReadPidParameters(void * pidAddr);
static void EEPROMWritePidParameters(void * pidAddr, PidParameters pidParameters);

void PIDConfSetDefault()
{
  g_gyroRollPidParameters = gyroDefaultPidParameters;
  g_gyroPitchPidParameters = gyroDefaultPidParameters;
  g_gyroYawPidParameters = gyroYawDefaultPidParameters;

  g_angleRollPidParameters = angleDefaultPidParameters;
  g_anglePitchPidParameters = angleDefaultPidParameters;
  g_angleYawPidParameters = angleDefaultPidParameters;
}

void PIDConfLoad()
{
  g_gyroRollPidParameters = EEPROMReadPidParameters(PID_ROLL_OMEGA_ADDR);
  g_gyroPitchPidParameters = EEPROMReadPidParameters(PID_PITCH_OMEGA_ADDR);
  g_gyroYawPidParameters = EEPROMReadPidParameters(PID_YAW_OMEGA_ADDR);

  g_angleRollPidParameters = EEPROMReadPidParameters(PID_ROLL_ANGLE_ADDR);
  g_anglePitchPidParameters = EEPROMReadPidParameters(PID_PITCH_ANGLE_ADDR);
  g_angleYawPidParameters = EEPROMReadPidParameters(PID_YAW_ANGLE_ADDR);
}

void PIDConfSave()
{
  EEPROMWritePidParameters(PID_ROLL_OMEGA_ADDR, g_gyroRollPidParameters);
  EEPROMWritePidParameters(PID_PITCH_OMEGA_ADDR, g_gyroPitchPidParameters);
  EEPROMWritePidParameters(PID_YAW_OMEGA_ADDR, g_gyroYawPidParameters);

  EEPROMWritePidParameters(PID_ROLL_ANGLE_ADDR, g_angleRollPidParameters);
  EEPROMWritePidParameters(PID_PITCH_ANGLE_ADDR, g_anglePitchPidParameters);
  EEPROMWritePidParameters(PID_YAW_ANGLE_ADDR, g_angleYawPidParameters);
}

static PidParameters EEPROMReadPidParameters(void * pidAddr)
{
  PidParameters resultPid;
  eeprom_read_block(&resultPid, pidAddr, sizeof(PidParameters));
  return resultPid;
}

static void EEPROMWritePidParameters(void * pidAddr, PidParameters pidParameters)
{
  eeprom_write_block(&pidParameters, pidAddr, sizeof(PidParameters));
}
#endif /* PIDCONF_H */
