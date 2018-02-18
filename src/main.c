#include "QuadState.h"
#include "core_pins.h"
#include "MCUConf.h"

static void mainLoop()
{
    int timeEnd;
    int timeStart = micros();

    /* Initial state */
    stateInit();
    while(1) {

        stateDo();
        stateUpdate();

        timeEnd = micros();
        /* Wait to match sampleTime */
        while((micros() - timeStart) < SAMPLE_TIME);
        timeStart = micros();
    }
}

void main() {
    mainLoop();
}


