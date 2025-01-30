/*
 * KalmanFilter_C.c
 *
 *  Created on: Jan 26, 2025
 *      Author: Neil
 */

#include "KalmanFilter_C.h"

void KalmanFilter_C (KalmanStruct* kState, float measurement){

	kState->p = kState->p + kState->q;
	kState->k = kState->p / (kState->p + kState->r);
	kState->x = kState->x + kState->k * (measurement - kState->x);
	kState->p = (1.0-kState->x) * kState->p;

}
