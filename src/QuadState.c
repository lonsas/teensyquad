#include "QuadState.h"
#include "stdbool.h"
#include "Receiver.h"
#include "Control.h"
#include "EscControl.h"
#include "Sensor.h"
#include "MCUConf.h"
#include "PIDConf.h"
#include "UsbCommunication.h"

#define TRANSITION(state) do { transition(state); return; } while(0)

/* Module local variables */
static void  (*pfState)();
static bool m_controlActive;
static bool m_sensorActive;
static bool m_usbActive;
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
    usbSetup();
    /*PIDConfLoad(); */
    PIDConfSetDefault();
    controlSetup();
    SensorSetup();
    EscControlSetup();

    m_sensorActive = true;
    m_controlActive = false;
    m_usbActive = false;

    entryDone(&stateStartupUpdate);
}

static void stateStartupUpdate()
{
    /* USB? */
    if(usbConnected()) {
        m_usbActive = true;
    }
    /* Transition */
    if(SensorOk() && receiverOk()) {
        TRANSITION(&stateReadyWait);
    }
}

static void stateReadyWait()
{
    tCurrState = READY_WAIT;
    prevArmSignal = receiverSignalHigh(AUX2);
    entryDone(&stateReadyWaitUpdate);
}

static void stateReadyWaitUpdate()
{

    /* Arm? */
    if(!prevArmSignal && receiverSignalHigh(AUX2)) {
        if(receiverSignalLow(THROTTLE) &&
           SensorOmegaIsZero()) {
            /* TODO: check all other controls that they are in the middle also */
            TRANSITION(&stateArmed);
        }
    }

    /* Update */
    prevArmSignal = receiverSignalHigh(AUX2);
}

static void stateArmed()
{
    tCurrState = ARMED;
    controlReset();
    EscControlArm();
    m_controlActive = true;

    entryDone(&stateArmedUpdate);
}

static void stateArmedUpdate()
{
    /* Disarm? */
    if(receiverSignalLow(AUX2) && receiverSignalLow(THROTTLE)) {
        EscControlDisarm();
        m_controlActive = false;
        TRANSITION(&stateReadyWait);
    }

    if(receiverSignalHigh(AUX1)) {
        controlSetAngleMode(true);
    } else {
        controlSetAngleMode(false);
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
    static unsigned int iteration = 0;
    if(m_sensorActive) {
        SensorUpdate();
    }
    if(m_controlActive) {
        doControl();
    }
    if(m_usbActive) {
        if(iteration % 1000 == 0) {
            usbUpdate();
        }
    }

    receiverFailSafe();

    iteration++;
}

void stateInit()
{
    m_sensorActive = false;
    m_controlActive = false;
    TRANSITION(&stateStartup);
}
