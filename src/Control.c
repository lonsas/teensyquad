#include "Control.h"
#include "GyroControl.h"
#include "AngleControl.h"
#include "mix.h"
#include "Receiver.h"
#include "EscControl.h"



void controlSetup()
{
    gyroControlSetup();
    angleControlSetup();
}

void controlReset()
{
    controlSetup();
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

    receiverGetControls(&dbThrottle, &dbRollRef, &dbPitchRef, &dbYawRef);
    setAngleRef(dbRollRef, dbPitchRef, dbYawRef);
    angleCalculateControl(&dbOmegaRoll, &dbOmegaPitch, &dbOmegaYaw);
    setOmegaRef(dbOmegaRoll, dbOmegaPitch, dbOmegaYaw);
    gyroCalculateControl(&dbOmegaDotRoll, &dbOmegaDotPitch, &dbOmegaDotYaw);

    mix(dbThrottle, 12, &dbOmegaDotRoll, &dbOmegaDotPitch, &dbOmegaDotYaw, output);
    EscControlOutput(output);
}



