#include "foc.h"

#include "bldcm_control.h"
#include "foc_utils.h"
#include "pid.h"

#include <stdio.h>

DQCurrent_s current;
DQVoltage_s voltage;
float shaft_angle;

PIDController * PID_current_q;
PIDController * PID_current_d;

void initFOC()
{
    PID_current_d = new_PIDController(0, 0, 0, 0, 0);
    PID_current_q = new_PIDController(0, 0, 0, 0, 0);
}

void loopFOC()
{
    shaft_angle = 0;
    current = GetFOCCurrents(shaft_angle);
    voltage.d = CalculatePID(PID_current_d, 0);
    voltage.q = CalculatePID(PID_current_q, 0);

    // calculate control phase voltage.
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
