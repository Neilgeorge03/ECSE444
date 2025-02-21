/*
 * measurement.h
 *
 *  Created on: Feb 19, 2025
 *      Author: neiljoegeorge
 */

#ifndef MEASUREMENT_H_
#define MEASUREMENT_H_

#include "stm32l4xx_hal.h"  // Include HAL library
#include "stm32l4xx_hal_adc.h"

#define VREFINT_ADDR  	((uint16_t*)0x1FFF75AA)
#define TS_CAL1_ADDR    ((uint16_t*)0x1FFF75A8) // 30°C
#define TS_CAL2_ADDR    ((uint16_t*)0x1FFF75CA) // 130°C
#define TS_CAL1_TEMP  30.0f                    // Temperature at TS_CAL1
#define TS_CAL2_TEMP  130.0f                   // Temperature at TS_CAL2

float getVoltage(ADC_HandleTypeDef *hadc1);
float getTemperature(ADC_HandleTypeDef *hadc1);


#endif
