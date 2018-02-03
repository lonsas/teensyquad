#ifndef PIDCONF_H
#define PIDCONF_H
#include "PID.h"

PidParameters g_tAngleParameters = {
    .K = 1,
    .Ti = 1,
    .Td = 0,
    .Tt = 1,
    .b = 1,
    .h = 1e-3,
    .N = 10,
    .limit = 1e9
};


PidParameters g_tGyroParameters = {
    .K = 1,
    .Ti = 1,
    .Td = 0,
    .Tt = 1,
    .b = 1,
    .h = 1e-3,
    .N = 10,
    .limit = 1e9
};

#endif /* PIDCONF_H */
