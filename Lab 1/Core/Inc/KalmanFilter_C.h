/*
 * KalmanFilter_C.h
 *
 *  Created on: Jan 26, 2025
 *      Author: Neil
 */

#ifndef INC_KALMANFILTER_C_H_
#define INC_KALMANFILTER_C_H_

#include "filter.h"
#include "math.h"
#include <float.h> // FLT_EPSILON


int KalmanFilter_C(KalmanStruct *kState, float measurement);

#endif /* INC_KALMANFILTER_C_H_ */
