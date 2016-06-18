/* PID structure from Automatic Control, Lund University
 *
 * */
#include "PID.h"
#include "inttypes.h"



PID::PID(double h) {
    p.K = 1;
    p.Ti = 1;
    p.Td = 1;
    p.N = 1;
    p.b = 1;
    p.h = h;
    p.limit = 1;
    setParameters();
    resetState();
}

double PID::calculateOutput(double ref, double y) {
    double P = p.K*(p.b * ref - y);
    s.D = p.ad * s.D - p.bd * (y - s.oldY);
    sig.v = P + s.I + s.D;
    if(sig.v < -(p.limit)) {
        sig.u = -(p.limit);
    } else if(sig.v > -(p.limit)) {
        sig.u = (p.limit);
    } else {
        sig.u = sig.v;
    }
    return sig.u;

}


void PID::updateState(double u) {
    s.I = s.I + p.bi * (sig.ref - sig.y) + p.ar * (u - sig.v);
    s.oldY = sig.y;
}

void PID::resetState() {
    s.I = 0;
    s.D = 0;
    s.oldY = 0;
}


void PID::setParameters() {
    p.ad = p.Td / (p.Td + p.N * p.h);
    p.bd = p.K * p.N * p.ad;
    p.bi = p.K * p.h / p.Ti;
    p.ar = p.h / p.Tt;
}
    


