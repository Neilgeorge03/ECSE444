/*
 * analysis.c
 *
 *  Created on: Feb 11, 2025
 *      Author: neiljoegeorge
 */

#include "analysis.h"

void CMSIS_analysis(float* TEST_ARRAY, float* result, int measurementCount){
    float difference[101];
    float mean, stdDev;
    float correlation[2 * measurementCount - 1];
    float convolution[2 * measurementCount - 1];

    // Difference
    arm_sub_f32(TEST_ARRAY, result, difference, measurementCount);

    // Mean and Standard Deviation
    arm_mean_f32(difference, measurementCount, &mean);
    arm_std_f32(difference, measurementCount, &stdDev);

    // Correlation
    arm_correlate_f32(TEST_ARRAY, measurementCount, result, measurementCount, correlation);

    // Convolution
    arm_conv_f32(TEST_ARRAY, measurementCount, result, measurementCount, convolution);
}

void C_analysis(float* TEST_ARRAY, float* result, int measurementCount){
    float difference[101];
    float mean = 0.0f, stdDev = 0.0f;
    float correlation[2 * measurementCount - 1];
    float convolution[2 * measurementCount - 1];

    // Difference
    for (int i = 0; i < measurementCount; i++) {
        difference[i] = TEST_ARRAY[i] - result[i];
    }

    // Mean
    for (int i = 0; i < measurementCount; i++) {
        mean += difference[i];
    }
    mean /= measurementCount;

    // Standard Deviation
    for (int i = 0; i < measurementCount; i++) {
        stdDev += (difference[i] - mean) * (difference[i] - mean);
    }
    stdDev = sqrtf(stdDev / measurementCount);

    // Correlation
    for (int lag = 0; lag < 2 * measurementCount - 1; lag++) {
        correlation[lag] = 0.0f;
        for (int i = 0; i < measurementCount; i++) {
            int j = lag + i - measurementCount + 1;
            if (j >= 0 && j < measurementCount) {
                correlation[lag] += TEST_ARRAY[i] * result[j];
            }
        }
    }

    // Convolution
    for (int i = 0; i < 2 * measurementCount - 1; i++) {
        convolution[i] = 0.0f;
        for (int j = 0; j < measurementCount; j++) {
            if (i >= j && (i - j) < measurementCount) {
                convolution[i] += TEST_ARRAY[j] * result[i - j];
            }
        }
    }
}
