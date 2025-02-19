/*
 * measurement.c
 *
 *  Created on: Feb 19, 2025
 *      Author: neiljoegeorge
 */


#include "main.h"
#include "measurement.h"
#include <stdint.h>


float getVoltage(ADC_HandleTypeDef *hadc) {
    uint32_t adc_value;
    uint16_t vrefint_cal = *VREFINT_ADDR;

    if (HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED) != HAL_OK) {
      Error_Handler();
    }

    HAL_ADC_Start(hadc);
    HAL_Delay(1);
    HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
    adc_value = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);

    float vRefPlus = (3.0f * vrefint_cal) / adc_value;

    return vRefPlus;
}

float getTemperature(ADC_HandleTypeDef *hadc) {
    uint32_t adc_value;
    uint16_t ts_cal1 = *TS_CAL1_ADDR;  // 30°C
    uint16_t ts_cal2 = *TS_CAL2_ADDR;  // 130°C

    // Read value from injected channel (which is responsible for TEMP)
    HAL_ADCEx_InjectedStart(hadc);
    HAL_Delay(1);
    adc_value = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);

    float vRef = getVoltage(hadc);  // Read actual VREF+
    float adj_adc_value = adc_value * (vRef / 3.0f);
    float temperature = ((100.0f / (float)(ts_cal2 - ts_cal1)) *
                         (adj_adc_value - ts_cal1)) + 30.0f;
    return temperature;
}
