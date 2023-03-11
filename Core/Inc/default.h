#ifndef __DEFAULT_H_
#define __DEFAULT_H_

#define DEF_POWER_SUPPLY   12.0  //!< default power supply voltage

#define DEF_PID_CURR_P     3                   //!< default PID controller P value
#define DEF_PID_CURR_I     300.0               //!<  default PID controller I value
#define DEF_PID_CURR_D     0.0                 //!<  default PID controller D value
#define DEF_PID_CURR_RAMP  0                   //!< default PID controller voltage ramp value
#define DEF_PID_CURR_LIMIT (DEF_POWER_SUPPLY)  //!< default PID controller voltage limit
#define DEF_CURR_FILTER_Tf 0.005               //!< default currnet filter time constant

#endif
