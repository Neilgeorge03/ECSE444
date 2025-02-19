#ifndef MEASUREMENT_H_
#define MEASUREMENT_H_

#include "stm32l4xx_hal.h"  // Include HAL library
#include "stm32l4xx_hal_adc.h"

float getVoltage(ADC_HandleTypeDef *hadc1);

float getTemperature(ADC_HandleTypeDef *hadc1);

#endif
