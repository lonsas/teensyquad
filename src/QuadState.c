#include "QuadState.h"
#include "stdbool.h"
#include "Receiver.h"
#include "Control.h"
#include "EscControl.h"
#include "Sensor.h"
#include "MCUConf.h"


/* Module local variables */
static void  (*pfState)();
static bool controlActive;
static bool sensorActive;
static bool prevArmSignal;
static enum state tCurrState;

/* State management functions */
static void entryDone(void(*pfUpdateState)())
{
    pfState = pfUpdateState;
}

static void transition(void(*pfNextStateEntry)())
{
    pfNextStateEntry();
}

/* State machine states */
static void stateStartup();
static void stateReadyWait();
static void stateArmed();

/* Update functions */
static void stateStartupUpdate();
static void stateReadyWaitUpdate();
static void stateArmedUpdate();

static void stateStartup()
{
    tCurrState = STARTUP;
    receiverSetup();
    controlSetup();
    SensorSetup();
    EscControlSetup();

    sensorActive = true;
    controlActive = false;

    entryDone(&stateStartupUpdate);
}

static void stateStartupUpdate()
{
    /* Transition */
    if(receiverOk() && SensorOk()) {
        transition(&stateReadyWait);
    }

}

static void stateReadyWait()
{
    tCurrState = READY_WAIT;
    prevArmSignal = receiverSignalHigh(AUX1);
    entryDone(&stateReadyWaitUpdate);
}

static void stateReadyWaitUpdate()
{
    /* Arm? */
    if(!prevArmSignal && receiverSignalHigh(AUX1)) {
        if(receiverSignalLow(THROTTLE) &&
           SensorAngleIsLevel() &&
           SensorOmegaIsZero()) {
            /* TODO: check all other controls that they are in the middle also */
            transition(&stateArmed);
        }
    }
    /* Update */
    prevArmSignal = receiverSignalHigh(AUX1);
}

static void stateArmed()
{
    tCurrState = ARMED;
    controlReset();
    EscControlArm();
    controlActive = true;

    entryDone(&stateArmedUpdate);
}

static void stateArmedUpdate()
{
    /* Disarm? */
    if(receiverSignalLow(AUX1)) {
        EscControlDisarm();
        controlActive = false;
        transition(&stateReadyWait);
    }
}

enum state getCurrState()
{
    return tCurrState;
}


void stateUpdate()
{
   pfState();
}

void stateDo()
{
        if(sensorActive) {
            SensorUpdate();
        }
        if(controlActive) {
            doControl();
        }
}

void stateInit()
{
    sensorActive = false;
    controlActive = false;
    transition(&stateStartup);
}
