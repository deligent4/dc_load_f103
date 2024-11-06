/*
 * filter.c
 *
 *  Created on: Nov 1, 2024
 *      Author: rahul
 */
#include <stdint.h>
#include "filter.h"

int Moving_Avg(int *ptrArrNumbers, long *ptrSum, int pos, int len, int nextNum)
{
  //Subtract the oldest number from the prev sum, add the new number
  *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
  //Assign the nextNum to the position in the array
  ptrArrNumbers[pos] = nextNum;
  //return the average
  return *ptrSum / len;
}


float Moving_Avg_Float(float *ptrArrNumbers, float *ptrSum, int pos, int len, float nextNum) {
    *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
    ptrArrNumbers[pos] = nextNum;
    return *ptrSum / len;
}


/*****************************************************
*Function name: kalman_filter
 *Function function: ADC_filter
 *Entry parameter: ADC_Value
 *Export parameters: kalman_adc
*****************************************************/
float Kalman_Filter(float ADC_Value){
    float x_k1_k1,x_k_k1;
    static float ADC_OLD_Value;
    float Z_k;
    static float P_k1_k1;

    static float Q = 0.01;//Q: Regulation noise, Q increases, dynamic response becomes faster, and convergence stability becomes worse
    static float R = 0.05; //R: Test noise, R increases, dynamic response becomes slower, convergence stability becomes better
    static float Kg = 0;
    static float P_k_k1 = 1;

    float kalman_adc;
    static float kalman_adc_old=0;
    Z_k = ADC_Value;
    x_k1_k1 = kalman_adc_old;

    x_k_k1 = x_k1_k1;
    P_k_k1 = P_k1_k1 + Q;

    Kg = P_k_k1/(P_k_k1 + R);

    kalman_adc = x_k_k1 + Kg * (Z_k - kalman_adc_old);
    P_k1_k1 = (1 - Kg)*P_k_k1;
    P_k_k1 = P_k1_k1;

//    ADC_OLD_Value = ADC_Value;
    kalman_adc_old = kalman_adc;

    return kalman_adc;
}
