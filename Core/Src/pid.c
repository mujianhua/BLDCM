#include "pid.h"

#include "foc_utils.h"
#include "stm32f4xx.h"  // Device header

#include <stdlib.h>

PIDController * new_PIDController(float _P, float _I, float _D, float _output_ramp, float _limit)
{
    PIDController * pObj = NULL;
    pObj = (PIDController *)malloc(sizeof(PIDController));
    pObj->P = _P;
    pObj->I = _I;
    pObj->D = _D;
    pObj->output_ramp = _output_ramp;
    pObj->limit = _limit;
    
    pObj->error_prev = 0;
    pObj->integral_prev = 0;
    pObj->output_prev = 0;
    pObj->timestamp_prev = 0;
    return pObj;
}

float CalculatePID(PIDController * const pid, float error)
{
    // calculate the time from the last call
    unsigned long timestamp_now = HAL_GetTick();
    float Ts = (timestamp_now - pid->timestamp_prev) * 1e-3;
    // quick fix for strange cases (micros overflow)
    if (Ts <= 0 || Ts > 0.5) Ts = 1e-3;

    // u(s) = (P + I/s + Ds)e(s)
    // Discrete implementations
    // proportional part
    // u_p  = P *e(k)
    float proportional = pid->P * error;
    // Tustin transform of the integral part
    // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
    float integral = pid->integral_prev + pid->I * Ts * 0.5 * (error + pid->error_prev);
    // antiwindup - limit the output
    integral = _constrain(integral, -pid->limit, pid->limit);
    // Discrete derivation
    // u_dk = D(ek - ek_1)/Ts
    float derivative = pid->D * (error - pid->error_prev) / Ts;

    // sum all the components
    float output = proportional + integral + derivative;
    // antiwindup - limit the output variable
    output = _constrain(output, -pid->limit, pid->limit);

    // if output ramp defined
    if (pid->output_ramp > 0) {
        // limit the acceleration by ramping the output
        float output_rate = (output - pid->output_prev) / Ts;
        if (output_rate > pid->output_ramp)
            output = pid->output_prev + pid->output_ramp * Ts;
        else if (output_rate < -pid->output_ramp)
            output = pid->output_prev - pid->output_ramp * Ts;
    }
    // saving for the next pass
    pid->integral_prev = integral;
    pid->output_prev = output;
    pid->error_prev = error;
    pid->timestamp_prev = timestamp_now;
    return output;
}
