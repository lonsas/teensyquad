#ifndef PIDCONF_H
#define PIDCONF_H
#include "PID.h"
#include "MCUConf.h"
#include "eeprom.h"

static const PidParameters gyroDefaultPidParameters = {
  .K = 1,
  .Ti = 10,
  .Td = 0,
  .Tt = 10,
  .b = 1,
  .h = 0.001,
};

static const PidParameters angleDefaultPidParameters = {
  .K = 1,
  .Ti = 10,
  .Td = 0,
  .Tt = 10,
  .b = 1,
  .h = 0.001,
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
  g_gyroYawPidParameters = gyroDefaultPidParameters;

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
