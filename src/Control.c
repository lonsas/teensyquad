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
    double throttle;
    double rollRef;
    double pitchRef;

    double dummy;
    double rollOmegaRef;
    double pitchOmegaRef;
    double yawOmegaRef;

    double rollOmegaDot;
    double pitchOmegaDot;
    double yawOmegaDot;

    double output[4];

    receiverGetControls(&throttle, &rollRef, &pitchRef, &yawOmegaRef);

    setAngleRef(rollRef, pitchRef, 0); /* No yaw angle control */
    angleCalculateControl(&rollOmegaRef, &pitchOmegaRef, &dummy);

    setOmegaRef(rollOmegaRef, pitchOmegaRef, yawOmegaRef);
    gyroCalculateControl(&rollOmegaDot, &pitchOmegaDot, &yawOmegaDot);

    mix(throttle, 12, &rollOmegaDot, &pitchOmegaDot, &yawOmegaDot, output);
    EscControlOutput(output);
}



