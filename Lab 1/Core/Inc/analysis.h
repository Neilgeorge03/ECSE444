/*
 * analysis.h
 *
 *  Created on: Feb 11, 2025
 *      Author: neiljoegeorge
 */

#ifndef INC_ANALYSIS_H_
#define INC_ANALYSIS_H_

#include "arm_math.h"
#include <math.h>       // For sqrtf


void CMSIS_analysis(float* TEST_ARRAY, float* result, int measurementCount);


void C_analysis(float* TEST_ARRAY, float* result, int measurementCount);


#endif /* INC_ANALYSIS_H_ */
