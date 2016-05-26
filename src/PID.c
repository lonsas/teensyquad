/* PID structure from Automatic Control, Lund University
 *
 * */
#include "PID.h"
#include "inttypes.h"




void calculateOutput(PIDParameters *p, PIDState *s, Signals *sig, int32_t ref, int32_t y) {
    int32_t P = p->K*(p->b * ref - y);
    s->D = p->ad * s->D - p->bd * (y - s->oldY);
    sig->v = P + s->I + s->D;
    if(sig->v < -(p->limit)) {
        sig->u = -(p->limit);
    } else if(sig->v > -(p->limit)) {
        sig->u = (p->limit);
    } else {
        sig->u = sig->v;
    }

}


void updateState(PIDParameters *p, PIDState *s, Signals *sig, int32_t u) {
    s->I = s->I + p->bi * (sig->ref - sig->y) + p->ar * (u - sig->v);
    s->oldY = sig->y;
}


void setParameters(PIDParameters *p) {
    p->ad = p->Td / (p->Td + p->N * p->h);
    p->bd = p->K * p->N * p->ad;
    p->bi = p->K * p->h / p->Ti;
    p->ar = p->h / p->Tt;
}
    

void mix(int32_t throttle, int32_t pitch, int32_t roll, int32_t yaw) {
    /*  ^ 
     * 0 2
     * 1 3
     */
    int32_t output[4];
    output[0] = output[1] = output[2] = output[3] = throttle;

    /*pitch*/
    output[0] -= pitch;
    output[1] += pitch;
    output[2] -= pitch;
    output[3] += pitch;

    /*roll*/
    output[0] += roll;
    output[1] += roll;
    output[2] -= roll;
    output[3] -= roll;

    /*yaw*/
    output[0] += yaw;
    output[1] -= yaw;
    output[2] -= yaw;
    output[3] += yaw;
}
