/*******************************************************************************
 * @file mcu_temp_hal_adc.c
 * @brief STM32L432KC internal temperature sense via ADC1_IN17.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "mcu_temp_hal_adc.h"
#include <math.h>

/** Private functions. ********************************************************/

/**
 * @brief Configure a single regular channel and run one blocking conversion.
 *
 * @param channel ADC channel to convert (e.g. ADC_CHANNEL_TEMPSENSOR).
 * @param out Pointer to store the raw conversion result.
 *
 * @return HAL_OK on success, HAL_ERROR otherwise.
 */
static HAL_StatusTypeDef mcu_temp_adc_read(uint32_t channel, uint32_t *out) {
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = MCU_TEMP_ADC_SAMPLINGTIME;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&MCU_TEMP_HADC, &sConfig) != HAL_OK) {
    return HAL_ERROR;
  }

  // Start a single conversion of the configured channel.
  if (HAL_ADC_Start(&MCU_TEMP_HADC) != HAL_OK) {
    return HAL_ERROR;
  }

  // Wait for the conversion to complete.
  if (HAL_ADC_PollForConversion(&MCU_TEMP_HADC, HAL_MAX_DELAY) != HAL_OK) {
    HAL_ADC_Stop(&MCU_TEMP_HADC);
    return HAL_ERROR;
  }

  *out = HAL_ADC_GetValue(&MCU_TEMP_HADC);
  HAL_ADC_Stop(&MCU_TEMP_HADC);
  return HAL_OK;
}

/** Public functions. *********************************************************/

HAL_StatusTypeDef mcu_temp_init(void) {
  // Run ADC single-ended self-calibration.
  return HAL_ADCEx_Calibration_Start(&MCU_TEMP_HADC, ADC_SINGLE_ENDED);
}

float get_mcu_temp(void) {
  uint32_t vrefint_data;
  uint32_t temp_data;

  // Measure VREFINT to derive the actual VDDA (Vref+) supply voltage, rather
  // than assuming a nominal 3.3 V rail.
  if (mcu_temp_adc_read(ADC_CHANNEL_VREFINT, &vrefint_data) != HAL_OK) {
    return NAN;
  }
  const uint32_t vdda_mv =
      __HAL_ADC_CALC_VREFANALOG_VOLTAGE(vrefint_data, MCU_TEMP_ADC_RESOLUTION);

  // Measure the internal temperature sensor channel.
  if (mcu_temp_adc_read(ADC_CHANNEL_TEMPSENSOR, &temp_data) != HAL_OK) {
    return NAN;
  }

  // Convert raw ADC data to degrees Celsius using the measured VDDA and the
  // factory temperature calibration values.
  return (float)__HAL_ADC_CALC_TEMPERATURE(vdda_mv, temp_data,
                                           MCU_TEMP_ADC_RESOLUTION);
}
