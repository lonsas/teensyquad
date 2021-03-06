#ifndef ANGLECONTROL_H
#define ANGLECONTROL_H


void setAngleRef(double dbRollAngleRef, double dbPitchAngleRef, double dbYawAngleRef);


void angleControlSetup();

void angleCalculateControl(double * pdbOmegaRollControl, \
                              double * pdbOmegaPitchControl, \
                              double * pdbOmegaYawControl);


void angleUpdate(double dbOmegaRollControlSat, double dbOmegaPitchControlSat, double dbOmegaYawControlSat);

#endif /* GYROCONTROL_H */
