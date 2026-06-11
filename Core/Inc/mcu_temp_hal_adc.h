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

// Analog reference voltage (Vref+) in mV, used to scale the ADC LSB.
#define MCU_TEMP_VREF_MV (3300u)

// ADC resolution used for the temperature conversion.
#define MCU_TEMP_ADC_RESOLUTION ADC_RESOLUTION_12B

/** Public functions. *********************************************************/

/**
 * @brief Get the current STM32L432KC internal temperature via ADC1_CH17.
 *
 * @return STM32L432KC internal temperature in degrees Celsius as float.
 * Returns NaN on conversion failure.
 */
float get_mcu_temp(void);

#endif
