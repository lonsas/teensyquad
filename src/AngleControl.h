#ifndef GYROCONTROL_H
#define GYROCONTROL_H


void setAngleRef(double dbRollAngleRef, double dbPitchAngleRef, double dbYawAngleRef);


void gyroControlSetup();

double dbAngleCalculateControl(double * pdbOmegaRollControl, \
                              double * pdbOmegaPitchControl, \
                              double * pdbOmegaYawControl);


void angleUpdate(double dbOmegaRollControlSat, double dbOmegaPitchControlSat, double dbOmegaYawControlSat);

#endif /* GYROCONTROL_H */
