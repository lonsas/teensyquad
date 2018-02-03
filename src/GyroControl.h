#ifndef GYROCONTROL_H
#define GYROCONTROL_H


void setOmegaRef(double dbRollOmegaRef, double dbPitchOmegaRef, double dbYawOmegaRef);


void gyroControlSetup();

void gyroCalculateControl(double * pdbOmegaDotRollControl, \
                              double * pdbOmegaDotPitchControl, \
                              double * pdbOmegaDotYawControl);


void gyroUpdate(double dbOmegaDotRollControlSat, double dbOmegaDotPitchControlSat, double dbOmegaDotYawControlSat);

#endif /* GYROCONTROL_H */
