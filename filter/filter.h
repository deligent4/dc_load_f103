/*
 * filter.h
 *
 *  Created on: Nov 1, 2024
 *      Author: Egoruch
 */

#ifndef FILTER_H_
#define FILTER_H_

int Moving_Avg(int *ptrArrNumbers, long *ptrSum, int pos, int len, int nextNum);
float Moving_Avg_Float(float *ptrArrNumbers, float *ptrSum, int pos, int len, float nextNum);
float Kalman_Filter(float ADC_Value);
#endif /* FILTER_H_ */
