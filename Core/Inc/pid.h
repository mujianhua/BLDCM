#ifndef PID_H
#define PID_H

#include "foc_utils.h"

typedef struct
{
    float P;
    float I;
    float D;
} PID;

#endif  // PID_H
