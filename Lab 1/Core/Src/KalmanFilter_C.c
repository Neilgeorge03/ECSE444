/*
 * KalmanFilter_C.c
 *
 *  Created on: Jan 26, 2025
 *      Author: Neil
 */

#include "KalmanFilter_C.h"

int KalmanFilter_C (float* InputArray, float* OutputArray, KalmanStruct* kState, int Length){

	for (int i = 0; i < Length; i++){
		kState->p = kState->p + kState->q;
		if (isinf(kState->p) || isnan(kState->p)) {
			return KALMAN_OVERFLOW; // Overflow error
		}
		if ((kState->p + kState->r) < FLT_EPSILON){
			return KALMAN_DIV_BY_ZERO;
		}
		kState->k = kState->p / (kState->p + kState->r);
		if (isinf(kState->k) || isnan(kState->k)) {
			return KALMAN_OVERFLOW; // Overflow error
		}

		kState->x = kState->x + kState->k * (InputArray[i] - kState->x);
		if (isinf(kState->x) || isnan(kState->x)) {
			return KALMAN_OVERFLOW; // Overflow error
		}

		kState->p = (1.0-kState->k) * kState->p;
		if (isinf(kState->p) || isnan(kState->p)) {
			return KALMAN_OVERFLOW; // Overflow error
		}
		OutputArray[i] = kState->x;
	}

    return KALMAN_SUCCESS;
}
