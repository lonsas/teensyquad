#ifndef DUMMY_HOST_FUNCTION_H
#define DUMMY_HOST_FUNCTION_H
#include <inttypes.h>

extern int pinArray[];
extern uint8_t usbOutputBuffer[];
extern uint8_t usbInputBuffer[];
extern unsigned int usbOutputEndIdx;
extern unsigned int usbOutputCurrIdx;
extern unsigned int usbInputEndIdx;
extern unsigned int usbInputCurrIdx;

void setMotion6(int ax, int ay, int az, int gx, int gy, int gz);

void getMotorOutput(int * motor);

#endif /* DUMMY_HOST_FUNCTION_H */


