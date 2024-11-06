/*
 * pid.c
 *
 *  Created on: May 14, 2024
 *      Author: phil
 *      https://github.com/pms67/PID/blob/master/PID.c
 */
#include "../../pid/pid.h"

uint16_t Control_DAC_Output(uint16_t mA_setpoint, int32_t mA_current, bool control_flag) {
    static uint16_t dac_value = 0;
    static int16_t previous_error = 0;   // Holds the error from the previous cycle for the derivative calculation
    static int32_t integral_sum = 0;     // Accumulates the integral of error over time

    int32_t mA_measured = abs(mA_current);
    int32_t error = mA_setpoint - mA_measured;
    const uint16_t dac_upper_limit = 40000;
    const uint16_t dac_lower_limit = 0;

    // Integer-scaled PID Control parameters
    int32_t Kp = 10;    // Proportional gain, scaled by 0
    int32_t Ki = 600;     // Integral gain, scaled by 1000
    int32_t Kd = 10;     // Derivative gain, scaled by 1000

    // Proportional Term (P)
    int32_t adjustment = (Kp * error);

    if(!(control_flag)){ // set all static to 0
    	previous_error = 0;
    	integral_sum = 0;
    	return dac_value = 0;
	}

//    if (control_flag) {
//        // Ramp up/down logic for smooth transition
//        if (dac_value < target_dac_value) {
//            dac_value += ramp_increment;
//            if (dac_value > target_dac_value) dac_value = target_dac_value;
//        } else if (dac_value > target_dac_value) {
//            dac_value -= ramp_increment;
//            if (dac_value < target_dac_value) dac_value = target_dac_value;
//        }
//    }

    // Integral Term (I)
    integral_sum += error;
    // Integral windup protection
    if (integral_sum > 10000) integral_sum = 10000;
    if (integral_sum < -10000) integral_sum = -10000;
    adjustment += (Ki * integral_sum) / 1000;

    // Derivative Term (D)
    int32_t derivative = error - previous_error;
    adjustment += (Kd * derivative) / 1000;

    // Calculate new DAC value and ensure it stays within limits
    int new_dac_value = (int)dac_value + adjustment;

    if (new_dac_value >= dac_upper_limit) {
        dac_value = dac_upper_limit;
    } else if (new_dac_value <= dac_lower_limit) {
        dac_value = dac_lower_limit;
    } else {
        dac_value = new_dac_value;
    }

    // Update previous error for the next cycle
    previous_error = error;

    // Print debug info
    printf("new_dac: %d, _dac: %d, error: %ld, measured_curr: %ld, integral: %ld, derivative: %ld\n\r\v",
           new_dac_value, dac_value, error, mA_measured, integral_sum, derivative);

    return dac_value;
}
