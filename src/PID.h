#ifndef __PID_H
#define __PID_H

#include "inttypes.h"

typedef struct PIDParameters {
    double K;
    double Ti;
    double Td;
    double Tt;
    double N;
    double b;
    double h;
    double limit;
    double ad, bd, bi, ar;
} PIDParameters;


typedef struct {
    double D;
    double I;
    double oldY;
} PIDState;


typedef struct {
    double ref;
    double y;
    double v;
    double u;
} Signals;


class PID {
private:
    //Parameters
    PIDParameters p;

    //State
    PIDState s;


    //Signals

public:
    Signals sig;
    PID(double);
    double calculateOutput(double, double);
    void updateState(double);
    void setParameters();
    void resetState();
};


#endif

