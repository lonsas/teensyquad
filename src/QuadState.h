#ifndef QUAD_STATE_H
#define QUAD_STATE_H

enum state {
    STARTUP,
    READY_WAIT,
    ARMED,
}; 

void stateUpdate();

void stateDo();

void stateInit();

enum state getCurrState();

#endif /* QUAD_STATE_H */