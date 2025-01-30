/*
 * KalmanFilter_C_CMSIS.h
 *
 *  Created on: Jan 26, 2025
 *      Author: Neil
 */

#ifndef INC_KALMANFILTER_C_CMSIS_H_
#define INC_KALMANFILTER_C_CMSIS_H_
#include <arm_math.h>  // Include CMSIS-DSP library
#include <stdio.h>
#include "filter.h"

void KalmanFilter_C_CMSIS(KalmanStruct *kState, float32_t measurement);

#endif /* INC_KALMANFILTER_C_CMSIS_H_ */
