/*
 * KalmanFilter_C_CMSIS.c
 *
 *  Created on: Jan 26, 2025
 *      Author: Neil
 */

#include "KalmanFilter_C_CMSIS.h"

void KalmanFilter_C_CMSIS(KalmanStruct* kState, float measurement) {

	float temp1, temp2;
	arm_add_f32(*kState->p, *kState->q, *kState->p, 1);
	arm_add_f32(*kState->p, *kState->r, *temp1, 1);
	kState->k = kState->p / temp1;
	arm_sub_f32(measurement, *kState->x, temp1, 1);
	arm_mul_f32(*kState->k, *temp1, *temp2, 1);
	arm_add_f32(*kState->x, *temp2, *kState->x, 1);
	arm_sub_f32(1.0f, *kState->k, temp1, 1);
	arm_mul_f32(*temp1, *kState->p, *kState->p, 1);

}
