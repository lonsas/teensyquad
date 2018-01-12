#ifndef PID_H
#define PID_H

#include "PID_types.h"

struct Pid;
//typedef struct Pid Pid;

/* Create Pid object 
 *
 * tParameters parameter set to be used
 * returns configured Pid struct
 */
extern Pid tPidSetup(const PidParameters tParameters);

/* Calculate to output
 * ptPid Pid struct to use
 * ref referece value
 * y measured value
 *
 * returns control signal
 */
extern double dbCalculateOutput(Pid * ptPid, double ref, double y);

/* Updates the Pid state
 * ptPid Pid struct to use
 * ref reference value
 * y measured value
 * u controlled value (final saturated value)
 */
extern void updateState(Pid * ptPid, double ref, double y, double u);

/* Resets the Pid state
 * ptPid the Pid whose state to reset
 */
extern void resetState(Pid * ptPid);

#endif /* PID_H */

