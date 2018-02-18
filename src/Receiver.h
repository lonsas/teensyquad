#ifndef RECEIVER_H
#define RECEIVER_H

#include "stdbool.h"

#define ROLL 0
#define PITCH 1
#define THROTTLE 2
#define YAW 3
#define AUX1 4
#define AUX2 5

void receiverGetControls(double * throttle, double * roll, double * pitch, double * yaw);
void receiverGetAux(double * aux1, double * aux2);

bool receiverSignalHigh(int signal);
bool receiverSignalLow(int signal);
bool receiverSignalMiddle(int signal);

void receiverSetup();

bool receiverOk();

#endif /* RECEIVER_H */
