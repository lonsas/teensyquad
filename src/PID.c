/* PID structure from Automatic Control, Lund University
 *
 * */
#include "PID.h"
#include <float.h>

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

double dbCalculateOutput(Pid * ptPid, double ref, double y)
{
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
          tParameters->bd * (y - tState->y);
    dbOutput = dbP + tState->I + dbD;

    /* Save state */
    tState->D = dbD;
    tState->u = dbOutput;
    tState->ref = ref;
    tState->y = y;

    if(dbOutput > tParameters->limit) {
        dbOutputSat = tParameters->limit;
    } else if(dbOutput < -tParameters->limit) {
        dbOutputSat = -tParameters->limit;
    } else {
        dbOutputSat = dbOutput;
    }
    return dbOutputSat;
}


void updateState(Pid * ptPid, double u)
{
    /* Forward approximation */
    ptPid->tState.I += ptPid->tParameters.bi * (ptPid->tState.ref - ptPid->tState.y) + \
                      ptPid->tParameters.ar * (u - ptPid->tState.u);
}

double dbCalculateAndUpdate(Pid * pPid, double ref, double y, double old_u)
{
    /* Update the state before to be able to get the saturated control signal */
    updateState(pPid, old_u);
    return dbCalculateOutput(pPid, ref, y);
}


void resetState(Pid * ptPid)
{
    ptPid->tState.I = 0;
    ptPid->tState.D = 0;
    ptPid->tState.y = 0;
    ptPid->tState.u = 0;
    ptPid->tState.ref = 0;
}


PidInternalParameters tCalculateParameters(PidParameters ptParameters)
{
    double ad;
    double bd;
    double bi;
    double ar;
    PidInternalParameters tPidInternalParameters;

    /* Protect against division by zero if Td is not set */
    double ad_div = ptParameters.Td + ptParameters.N * ptParameters.h;
    if(ad_div != 0) {
        ad = ptParameters.Td / ad_div;
    } else {
        ad = 0;
    }

    bd = ptParameters.K * ptParameters.N * ad;

    /* Protect against division by zero, Ti == 0 is the same as Ti == inf */
    if(ptParameters.Ti != 0) {
        bi = ptParameters.K * ptParameters.h / ptParameters.Ti;
    } else {
        bi = 0;
    }

    /* Protect against division by zero, Tt == 0 is the same as Tt == inf */
    if(ptParameters.Tt != 0) {
        ar = ptParameters.h / ptParameters.Tt;
    } else {
        ar = 0;
    }

    if(ptParameters.limit == 0) {
        tPidInternalParameters.limit = DBL_MAX;
    } else {
        tPidInternalParameters.limit = ptParameters.limit;
    }


    tPidInternalParameters.ad = ad;
    tPidInternalParameters.bd = bd;
    tPidInternalParameters.bi = bi;
    tPidInternalParameters.ar = ar;
    tPidInternalParameters.K = ptParameters.K;
    tPidInternalParameters.b = ptParameters.b;

    return tPidInternalParameters;
}

double checkSignalDiff(Pid * ptPid, double u) {
  return ptPid->tState.u - u;
}

