#ifndef __FOC_H_
#define __FOC_H_

#include "default.h"
#include "foc_utils.h"

void initFOC();
void loopFOC();

DQCurrent_s GetFOCCurrents(float angle_el);

#endif
