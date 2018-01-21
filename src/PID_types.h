#ifndef PID_TYPES_H
#define PID_TYPES_H

typedef struct {
    double K;
    double Ti;
    double Td;
    double Tt;
    double b;
    double h;
    double N;
    double limit;
} PidParameters;

typedef struct {
    double K;
    double b;
    double limit;
    double ad;
    double bd;
    double bi;
    double ar;
} PidInternalParameters;

typedef struct {
    double D;
    double I;
    double y;
    double u;
    double ref;
} PidState;

typedef struct {
    PidInternalParameters tParameters;
    PidState tState;
} Pid;
#endif /* PID_TYPES_H */
