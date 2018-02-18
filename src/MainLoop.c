#include "QuadState.h"
#include "core_pins.h"
#include "MCUConf.h"

static int dt;

void mainLoop()
{
    int timeStart = micros();

    /* Initial state */
    stateInit();
    while(1) {

        stateDo();
        stateUpdate();

        dt = micros() - timeStart;
        /* Wait to match sampleTime */
        while((micros() - timeStart) < SAMPLE_TIME);
        timeStart = micros();
    }
}

int getDt()
{
    return dt;
}
