/*
 * pid.h
 *
 *  Created on: May 14, 2024
 *      Author: phil
 *      https://github.com/pms67/PID/blob/master/PID.c
 */

#ifndef INC_PID_H_
#define INC_PID_H_
#include <math.h>
#include "stdint.h"
#include "stdlib.h"
#include <stdio.h>
#include "stdbool.h"


uint16_t Control_DAC_Output(uint16_t mA_setpoint, int32_t voltage_on_load, bool control_flag);



#endif /* INC_PID_H_ */
