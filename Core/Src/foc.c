#include "foc.h"

#include "bldcm_control.h"
#include "foc_utils.h"
#include "main.h"
#include "pid.h"

#include <stdio.h>

DQCurrent_s current;
DQVoltage_s voltage;
float shaft_angle;

PIDController * PID_current_q;
PIDController * PID_current_d;

uint16_t Tcmp1 = 0, Tcmp2 = 0, Tcmp3 = 0;

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

    float electrical_angle = 0;
    uint8_t sector = 0;

    AlphaBetaVoltage_s v_alphabeta = IPark(&voltage, electrical_angle);

    uint16_t T1, T2, T3;
    uint16_t Ta, Tb, Tc;
    uint16_t X = _SQRT3 * v_alphabeta.beta / UDC * PWM_TIM_Counter;
    uint16_t Y = (3 / 2 * v_alphabeta.alpha + _SQRT3_2 * v_alphabeta.beta) / UDC * PWM_TIM_Counter;
    uint16_t Z = (-3 / 2 * v_alphabeta.alpha + _SQRT3_2 * v_alphabeta.beta) / UDC * PWM_TIM_Counter;
    switch (sector) {
        case 1:
            T1 = Z;
            T2 = Y;
            break;
        case 2:
            T1 = Y;
            T2 = -X;
            break;
        case 3:
            T1 = -Z;
            T2 = X;
            break;
        case 4:
            T1 = -X;
            T2 = Z;
            break;
        case 5:
            T1 = X;
            T2 = -Y;
            break;
        default:
            T1 = -Y;
            T2 = -Z;
            break;
    }
    float f_temp = T1 + T2;
    if (f_temp > PWM_TIM_Counter) {
        T1 = T1 / f_temp * PWM_TIM_Counter;
        T2 = T2 / f_temp * PWM_TIM_Counter;
    }
    Ta = (PWM_TIM_Counter - (T1 + T2)) / 4.0;
    Tb = Ta + T1 / 2;
    Tc = Tb + T2 / 2;
    switch (sector) {
        case 1:
            Tcmp1 = Tb;
            Tcmp2 = Ta;
            Tcmp3 = Tc;
            break;
        case 2:
            Tcmp1 = Ta;
            Tcmp2 = Tc;
            Tcmp3 = Tb;
            break;
        case 3:
            Tcmp1 = Ta;
            Tcmp2 = Tb;
            Tcmp3 = Tc;
            break;
        case 4:
            Tcmp1 = Tc;
            Tcmp2 = Tb;
            Tcmp3 = Ta;
            break;
        case 5:
            Tcmp1 = Tc;
            Tcmp2 = Ta;
            Tcmp3 = Tb;
            break;
        case 6:
            Tcmp1 = Tb;
            Tcmp2 = Tc;
            Tcmp3 = Ta;
            break;

        default:
            Tcmp1 = 0;
            Tcmp2 = 0;
            Tcmp3 = 0;
            break;
    }
}

AlphaBetaVoltage_s IPark(const DQVoltage_s * v_dq, float theta)
{
    AlphaBetaVoltage_s output;
    output.alpha = _cos(theta) * v_dq->d - _sin(theta) * v_dq->q;
    output.beta = _sin(theta) * v_dq->d + _cos(theta) * v_dq->q;
    return output;
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
