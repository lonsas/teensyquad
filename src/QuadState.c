#include "QuadState.h"
#include "stdbool.h"
#include "Receiver.h"
#include "Control.h"
#include "EscControl.h"
#include "Sensor.h"
#include "MCUConf.h"
#include "PIDConf.h"
#include "UsbCommunication.h"


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
static void stateUsbConnected();

/* Update functions */
static void stateStartupUpdate();
static void stateReadyWaitUpdate();
static void stateArmedUpdate();
static void stateUsbConnectedUpdate();

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

    entryDone(&stateStartupUpdate);
}

static void stateStartupUpdate()
{
    /* Transition */
    if(SensorOk()) {
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
    /* USB? */
    if(usbConnected() || 1) {
      transition(&stateUsbConnected);
    }
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
    m_controlActive = true;

    entryDone(&stateArmedUpdate);
}

static void stateArmedUpdate()
{
    /* Disarm? */
    if(receiverSignalLow(AUX1) && receiverSignalLow(THROTTLE)) {
        EscControlDisarm();
        m_controlActive = false;
        transition(&stateReadyWait);
    }
}

static void stateUsbConnected()
{
    tCurrState = USB_CONNECTED;
    m_usbActive = true;
    entryDone(&stateUsbConnectedUpdate);
}

static void stateUsbConnectedUpdate()
{
    if(!usbConnected())
    {
        m_usbActive = false;
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
    static unsigned int iteration = 0;
    if(m_sensorActive) {
        SensorUpdate();
    }
    if(m_controlActive) {
        doControl();
    }
    if(m_usbActive) {
        if(iteration % 10 == 0) {
            usbUpdate();
        }
    }
}

void stateInit()
{
    m_sensorActive = false;
    m_controlActive = false;
    transition(&stateStartup);
}
