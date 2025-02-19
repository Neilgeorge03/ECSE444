/*
 * KalmanFilter_C.c
 *
 *  Created on: Jan 26, 2025
 *      Author: Neil
 */

#include "KalmanFilter_C.h"

int KalmanFilter_C (kalman_state* kState, float measurement){

	kState->p = kState->p + kState->q;
    if (isinf(kState->p) || isnan(kState->p)) {
        return KALMAN_OVERFLOW; // Overflow error
    }

    float denominator = kState->p + kState->r;
    if (denominator < FLT_EPSILON) {
        return KALMAN_DIV_BY_ZERO;
    }

    // Update: Calculate the Kalman gain
    kState->k = kState->p / denominator;


    if (isinf(kState->k) || isnan(kState->k)) {
        return KALMAN_OVERFLOW; // Overflow error
    }

	kState->x += kState->k * (measurement - kState->x);
    if (isinf(kState->x) || isnan(kState->x)) {
        return KALMAN_OVERFLOW; // Overflow error
    }

	kState->p *= (1.0f-kState->k);
    if (isinf(kState->p) || isnan(kState->p)) {
        return KALMAN_OVERFLOW; // Overflow error
    }

    return KALMAN_SUCCESS;
}
