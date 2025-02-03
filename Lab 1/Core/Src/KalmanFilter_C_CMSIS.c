/*
 * KalmanFilter_C_CMSIS.c
 *
 *  Created on: Jan 26, 2025
 *      Author: Neil
 */

#include "KalmanFilter_C_CMSIS.h"

int KalmanFilter_C_CMSIS(KalmanStruct* kState, float measurement) {
    float32_t temp1[1], temp2[1], one[1] = {1.0f};

    // P = P + Q
    arm_add_f32(&(kState->p), &(kState->q), &(kState->p), 1);
    if (isinf(kState->p) || isnan(kState->p)) {
        return KALMAN_OVERFLOW; // Overflow error
    }
    if ((kState->p + kState->r) < FLT_EPSILON){
    	return KALMAN_DIV_BY_ZERO;
    }

    // temp1 = P + R
    arm_add_f32(&(kState->p), &(kState->r), temp1, 1);

    // K = P / temp1
    kState->k = kState->p / temp1[0];

    if (isinf(kState->k) || isnan(kState->k)) {
        return KALMAN_OVERFLOW; // Overflow error
    }

    // temp1 = measurement - X
    arm_sub_f32(&measurement, &(kState->x), temp1, 1);

    // temp2 = K * temp1
    arm_mult_f32(&(kState->k), temp1, temp2, 1);

    // X = X + temp2
    arm_add_f32(&(kState->x), temp2, &(kState->x), 1);
    if (isinf(kState->x) || isnan(kState->x)) {
        return KALMAN_OVERFLOW; // Overflow error
    }

    // temp1 = 1 - K
    arm_sub_f32(one, &(kState->k), temp1, 1);

    // P = temp1 * P
    arm_mult_f32(temp1, &(kState->p), &(kState->p), 1);

    if (isinf(kState->p) || isnan(kState->p)) {
        return KALMAN_OVERFLOW; // Overflow error
    }

    return KALMAN_SUCCESS;
}
