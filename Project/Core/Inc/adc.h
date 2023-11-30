/*
 * adc.h
 *
 *  Created on: Nov 30, 2023
 *      Author: juang
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include <stdint.h>
#include "main.h"


uint16_t init_adc(ADC_HandleTypeDef *hadc);
uint16_t calculateBPM();

#endif /* INC_ADC_H_ */
