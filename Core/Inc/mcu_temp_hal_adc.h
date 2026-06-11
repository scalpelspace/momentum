/*******************************************************************************
 * @file mcu_temp_hal_adc.h
 * @brief STM32L432KC internal temperature sense via ADC1_IN17.
 *******************************************************************************
 */

#ifndef MOMENTUM__MCU_TEMP_HAL_ADC_H
#define MOMENTUM__MCU_TEMP_HAL_ADC_H

/** Includes. *****************************************************************/

#include "stm32l4xx_hal.h"

/** STM32 port and pin configs. ***********************************************/

extern ADC_HandleTypeDef hadc1;

// ADC.
#define MCU_TEMP_HADC hadc1

/** Definitions. **************************************************************/

// ADC resolution used for the conversions.
#define MCU_TEMP_ADC_RESOLUTION ADC_RESOLUTION_12B

// Sampling time for the internal channels. Both the temperature sensor and
// VREFINT require a long minimum sampling time.
#define MCU_TEMP_ADC_SAMPLINGTIME ADC_SAMPLETIME_640CYCLES_5

/** Public functions. *********************************************************/

/**
 * @brief Initialize the MCU temperature sense ADC.
 *
 * Runs the ADC single-ended self-calibration sequence. Must be called once,
 * after the ADC peripheral is initialized (MX_ADC1_Init) and before the first
 * call to get_mcu_temp(). Calibration requires the ADC to be disabled, so this
 * must not be called while a conversion is in progress.
 *
 * @return HAL_OK on success, HAL error status otherwise.
 */
HAL_StatusTypeDef mcu_temp_init(void);

/**
 * @brief Get the current STM32L432KC internal temperature via ADC1_CH17.
 *
 * @return STM32L432KC internal temperature in degrees Celsius as float.
 * @retval == NaN -> conversion failure.
 */
float get_mcu_temp(void);

#endif
