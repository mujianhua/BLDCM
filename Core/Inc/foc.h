#ifndef __FOC_H_
#define __FOC_H_

#include "foc_utils.h"

void loopFOC();

DQCurrent_s GetFOCCurrents(float angle_el);

#endif
