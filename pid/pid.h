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

typedef struct {

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;

	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;

} PIDController;

void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);
uint16_t Control_DAC_Output(uint16_t mA_setpoint, int32_t voltage_on_load, bool control_flag);
#endif /* INC_PID_H_ */
