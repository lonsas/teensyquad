#include "control.h"




void controlSetup()
{
    gyroControlSetup();
    angleControlSetup();
}


void doControl() {
    double dbThrottle;
    double dbRollRef;
    double dbPitchRef;
    double dbYawRef;

    double dbOmegaRoll;
    double dbOmegaPitch;
    double dbOmegaYaw;
    
    double dbOmegaDotRoll;
    double dbOmegaDotPitch;
    double dbOmegaDotYaw;

    double output[4];

    radioInputGet(&throttle, &dbRollRef, &dbPitchRef, &dbYawRef);
    setAngleRef(dbRollRef, dbPitchRef, dbYawRef);
    dbAngleCalculateControl(&dbOmegaRoll, &dbOmegaPitch, &dbOmegaYaw);
    setOmegaRef(dbOmegaRoll, dbOmegaPitch, dbOmegaYaw);
    dbGyroCalculateControl(&dbOmegaDotRoll, &dbOmegaDotPitch, &dbOmegaDotYaw);

    mix(throttle, 12, &dbOmegaDotRoll, &dbOmegaDotPitch, &dbOmegaDotYaw, output);

}



