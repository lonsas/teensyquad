#ifndef __PID_H
#define __PID_H

#include "inttypes.h"

typedef struct PIDParameters {
    int32_t K;
    int32_t Ti;
    int32_t Td;
    int32_t Tt;
    int32_t N;
    int32_t b;
    int32_t h;
    int32_t limit;
    int32_t ad, bd, bi, ar;
} PIDParameters;


typedef struct {
    int32_t D;
    int32_t I;
    int32_t oldY;
} PIDState;


typedef struct {
    int32_t ref;
    int32_t y;
    int32_t v;
    int32_t u;
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
    PID(int32_t);
    int32_t calculateOutput(int32_t, int32_t);
    void updateState(int32_t);
    void setParameters();
    void resetState();
};


#endif

