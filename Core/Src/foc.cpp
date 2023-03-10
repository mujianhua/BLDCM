#include "foc.h"

#include "bldcm_control.h"
#include "foc_utils.h"

void loopFOC() {
    DQCurrent_s current = GetFOCCurrents(0);
}

DQCurrent_s GetFOCCurrents(float angle_el)
{
    PhaseCurrent_s current = GetPhaseCurrents();
    // calculate clarke transform
    float i_alpha, i_beta;
    if (!current.w) {
        // if only two measured currents
        i_alpha = current.u;
        i_beta = _1_SQRT3 * current.u + _2_SQRT3 * current.v;
    } else {
        // signal filtering using identity a + b + c = 0. Assumes measurement error is normally
        // distributed.
        float mid = (1.f / 3) * (current.u + current.v + current.w);
        float u = current.u - mid;
        float v = current.v - mid;
        i_alpha = u;
        i_beta = _1_SQRT3 * u + _2_SQRT3 * v;
    }
	// calculate park transform
    float ct = _cos(angle_el);
    float st = _sin(angle_el);
    DQCurrent_s return_current;
    return_current.d = i_alpha * ct + i_beta * st;
    return_current.q = i_beta * ct - i_alpha * st;
    return return_current;
}
