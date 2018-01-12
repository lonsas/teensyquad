/* PID structure from Automatic Control, Lund University
 *
 * */
#include "PID.h"

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
    double oldY;
    double u;
} PidState;

struct Pid {
    PidInternalParameters tParameters;
    PidState tState;
};

/* Precalculates some recurring values in the parameter set
 * ptParameters the parameter set that should be configured
 */
static PidInternalParameters tCalculateParameters(PidParameters ptParameters);

Pid tPidSetup(PidParameters tParameters)
{
    Pid tPid;
    tPid.tParameters = tCalculateParameters(tParameters);
    resetState(&tPid);
    return tPid;
} 

double dbCalculateOutput(Pid * ptPid, double ref, double y) {
    double dbP;
    double dbD;
    double dbOutput;
    double dbOutputSat;
    PidState * tState;
    PidInternalParameters * tParameters;
    
    tState = &(ptPid->tState);
    tParameters = &(ptPid->tParameters);

    /* Calculate */
    dbP = tParameters->K * (tParameters->b * ref - y);
    dbD = tParameters->ad * tState->D - \
          tParameters->bd * (y - tState->oldY);
    dbOutput = dbP + tState->I + dbD;

    /* Save state */
    tState->D = dbD;
    tState->u = dbOutput;

    if(dbOutput > tParameters->limit) {
        dbOutputSat = tParameters->limit;
    } else if(dbOutput < -tParameters->limit) {
        dbOutputSat = -tParameters->limit;
    } else {
        dbOutputSat = dbOutput;
    }
    return dbOutputSat;
}


void updateState(Pid * ptPid, double ref, double y, double u) {
    ptPid->tState.I += ptPid->tParameters.bi * (ref - y) + \
                      ptPid->tParameters.ar * (ptPid->tState.u - u);
    ptPid->tState.oldY = y;
}

void resetState(Pid * ptPid) {
    ptPid->tState.I = 0;
    ptPid->tState.D = 0;
    ptPid->tState.oldY = 0;
    ptPid->tState.u = 0;
}


PidInternalParameters tCalculateParameters(PidParameters ptParameters) {
    double ad;
    double bd;
    double bi;
    double ar;
    PidInternalParameters tPidInternalParameters;

    ad = ptParameters.Td / \
                       (ptParameters.Td + ptParameters.N * ptParameters.h);
    bd = ptParameters.K * ptParameters.N * ad;
    bi = ptParameters.K * ptParameters.h / ptParameters.Ti;
    ar = ptParameters.h / ptParameters.Tt;


    tPidInternalParameters.ad = ad;
    tPidInternalParameters.bd = bd;
    tPidInternalParameters.bi = bi;
    tPidInternalParameters.ar = ar;
    tPidInternalParameters.K = ptParameters.K;
    tPidInternalParameters.b = ptParameters.b;
    tPidInternalParameters.limit = ptParameters.limit;

    return tPidInternalParameters;
}
    


