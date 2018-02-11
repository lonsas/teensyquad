#ifndef SENSOR_H
#define SENSOR_H

void SensorGetOmega(double * pdbRollOmega, double * pdbPitchOmega, double * pdbYawOmega);

void SensorGetAngle(double * pdbRollAngle, double * pdbPitchAngle, double * pdbYawAngle);

void SensorSetup();

void SensorUpdate();

#endif /* SENSOR_H */
