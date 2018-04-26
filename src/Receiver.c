#include "Receiver.h"
#include "inttypes.h"
#include "core_pins.h"

/* Constants */
#define RADIO_PINS 6
#define SIGNAL_MAX 2100
#define SIGNAL_MIN 900

#define TIMEOUT 1000000

const static uint8_t RADIOPIN[RADIO_PINS] = {2,3,4,5,6,7};
/* Make sure the RISE pin is on a failsafe signal i.e throttle */
const static uint8_t RADIOPIN_RISE = 1;

/* Interrupt managed variables */
static volatile int32_t radio_rise = -(TIMEOUT);
static volatile int32_t width[RADIO_PINS];

/* Interrupts */
void radio_pw_rise_isr() {
    radio_rise = micros();
}

void radio_pw0_isr() {
    width[0] = micros() - radio_rise;
}
void radio_pw1_isr() {
    width[1] = micros() - radio_rise;
}
void radio_pw2_isr() {
    width[2] = micros() - radio_rise;
}
void radio_pw3_isr() {
    width[3] = micros() - radio_rise;
}
void radio_pw4_isr() {
    width[4] = micros() - radio_rise;
}
void radio_pw5_isr() {
    width[5] = micros() - radio_rise;
}

/* Global functions */
void receiverSetup()
{
    /* Uninitialized */
    width[0] = -1;
    width[1] = -1;
    width[2] = -1;
    width[3] = -1;
    width[4] = -1;
    width[5] = -1;

    pinMode(RADIOPIN_RISE, INPUT);
    for(int i = 0; i < RADIO_PINS; i++) {
        pinMode(RADIOPIN[i], INPUT);
    }
    attachInterrupt(RADIOPIN_RISE, radio_pw_rise_isr, RISING);
    attachInterrupt(RADIOPIN[0], radio_pw0_isr, FALLING);
    attachInterrupt(RADIOPIN[1], radio_pw1_isr, FALLING);
    attachInterrupt(RADIOPIN[2], radio_pw2_isr, FALLING);
    attachInterrupt(RADIOPIN[3], radio_pw3_isr, FALLING);
    attachInterrupt(RADIOPIN[4], radio_pw4_isr, FALLING);
    attachInterrupt(RADIOPIN[5], radio_pw5_isr, FALLING);
}

void receiverGetControls(double * throttle, double * roll, double * pitch, double * yaw)
{
    /* TODO Disable interrupt */
    *roll = (width[ROLL] - 1500) / 500.0;
    *pitch = (width[PITCH] - 1500) / 500.0;
    *yaw = (width[YAW] - 1500) / 500.0;
    *throttle = (width[THROTTLE] - 1000) / 1000.0;

}

void receiverGetAux(double * aux1, double * aux2)
{
    *aux1 = (width[AUX1] - 1000) / 1000.0;
    *aux2 = (width[AUX2] - 1000) / 1000.0;
}

bool receiverOk()
{
    bool signalRangeOk = true;
    /* TODO: Fix micros() overflow */
    bool newSignals = ((micros() - radio_rise) < TIMEOUT);
    for(int i = 0; i < RADIO_PINS; i++)
    {
        if((width[i] < SIGNAL_MIN) || (width[i] > SIGNAL_MAX)) {
            signalRangeOk = false;
            break;
        }
    }
    return (newSignals && signalRangeOk);
}

bool receiverSignalHigh(int signal)
{
    return (width[signal] > 1900);
}

bool receiverSignalLow(int signal)
{
    return (width[signal] < 1100);
}

bool receiverSignalMiddle(int signal)
{
    return ((width[signal] < 1600) && (width[signal] > 1400));
}

void receiverSetAllManualPW(int * micros) {
    for(int i = 0; i < RADIO_PINS; i++) {
        width[i] = micros[i];
    }
}

void receiverSetManualPW(int signal, int micros) {
    width[signal] = micros;
}
