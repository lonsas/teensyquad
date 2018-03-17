#include "Control.h"
#include "GyroControl.h"
#include "AngleControl.h"
#include "mix.h"
#include "Receiver.h"
#include "EscControl.h"

#ifndef PI
#define PI 3.1415
#endif

#define MAX_ANGLE PI*0.25
#define MAX_OMEGA PI*2

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

    /* Convert the receiver controls to relevant units */
    /* TODO: Make the scalings tunable */
    rollRef *= MAX_ANGLE;
    pitchRef *= MAX_ANGLE;
    yawOmegaRef *= MAX_OMEGA;

    setAngleRef(rollRef, pitchRef, 0); /* No yaw angle control */
    angleCalculateControl(&rollOmegaRef, &pitchOmegaRef, &dummy);

    setOmegaRef(rollOmegaRef, pitchOmegaRef, yawOmegaRef);
    gyroCalculateControl(&rollOmegaDot, &pitchOmegaDot, &yawOmegaDot);

    mix(throttle, 12, &rollOmegaDot, &pitchOmegaDot, &yawOmegaDot, output);
    EscControlOutput(output);
}

