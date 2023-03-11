#ifndef PID_H
#define PID_H

#include "foc_utils.h"

typedef struct
{
    float P;                       //!< Proportional gain
    float I;                       //!< Integral gain
    float D;                       //!< Derivative gain
    float output_ramp;             //!< Maximum speed of change of the output value
    float limit;                   //!< Maximum output value
    float integral_prev;           //!< last integral component value
    float error_prev;              //!< last tracking error value
    unsigned long timestamp_prev;  //!< Last execution timestamp
    float output_prev;             //!< last pid output value
} PIDController;

PIDController * new_PIDController(float _P, float _I, float _D, float _output_ramp, float _limit);

float CalculatePID(PIDController * const pid, float error);

#endif  // PID_H
