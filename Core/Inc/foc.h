#ifndef __FOC_H_
#define __FOC_H_

#include "default.h"
#include "foc_utils.h"

// extern uint16_t Tcmp1, Tcmp2, Tcmp3;

void initFOC();
void loopFOC();

AlphaBetaVoltage_s IPark(const DQVoltage_s * v_dq, float theta);

DQCurrent_s GetFOCCurrents(float angle_el);

#endif
