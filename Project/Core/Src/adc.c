#include "adc.h"


uint16_t adc_reading=0;
uint16_t BPM_value=0;

uint32_t beat_old = 0;
float beats[500] = {0};
uint32_t beatIndex = 0;


uint16_t init_adc(ADC_HandleTypeDef *hadc){
	  HAL_ADC_Start(hadc);
	  HAL_ADC_PollForConversion(hadc, 20);
	  adc_reading = HAL_ADC_GetValue(hadc);
	  return adc_reading;
}


uint16_t calculateBPM() {
  uint32_t beat_new= HAL_GetTick();
  uint32_t diff = beat_new - beat_old; // find the time between the last two beats

  float currentBPM = 60000.0 / diff; // convert to beats per minute
  beats[beatIndex] = currentBPM; // store to array to compute the average

  float total = 0.0;
  for (int i = 0; i < 500; i++) {
    total += beats[i];
  }

  BPM_value = total / 500; // Compute average BPM


  beat_old = beat_new;
  beatIndex = (beatIndex + 1) % 500; // cycle through the array

  return BPM_value;
}

