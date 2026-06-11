/*******************************************************************************
 * @file mcu_temp_hal_adc.c
 * @brief STM32L432KC internal temperature sense via ADC1_IN17.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "mcu_temp_hal_adc.h"
#include <math.h>

/** Public functions. *********************************************************/

HAL_StatusTypeDef mcu_temp_init(void) {
  // Run ADC single-ended self-calibration.
  return HAL_ADCEx_Calibration_Start(&MCU_TEMP_HADC, ADC_SINGLE_ENDED);
}

float get_mcu_temp(void) {
  uint32_t adc_data;

  // Start a single conversion of the internal temperature channel.
  if (HAL_ADC_Start(&MCU_TEMP_HADC) != HAL_OK) {
    return NAN;
  }

  // Wait for the conversion to complete.
  if (HAL_ADC_PollForConversion(&MCU_TEMP_HADC, HAL_MAX_DELAY) != HAL_OK) {
    HAL_ADC_Stop(&MCU_TEMP_HADC);
    return NAN;
  }

  adc_data = HAL_ADC_GetValue(&MCU_TEMP_HADC);

  HAL_ADC_Stop(&MCU_TEMP_HADC);

  // Convert raw ADC data to degrees Celsius using factory calibration values.
  return (float)__HAL_ADC_CALC_TEMPERATURE(MCU_TEMP_VREF_MV, adc_data,
                                           MCU_TEMP_ADC_RESOLUTION);
}
