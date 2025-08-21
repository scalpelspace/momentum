/*******************************************************************************
 * @file bmp5xx_runner.h
 * @brief BMP5xx runner: init and data reading.
 *******************************************************************************
 */

#ifndef MOMENTUM__BMP5xx_RUNNER_H
#define MOMENTUM__BMP5xx_RUNNER_H

/** Includes. *****************************************************************/

#include "bmp5.h"
#include "bmp5_hal_i2c.h"

/** Public variables. *********************************************************/

extern float bmp5xx_temperature;
extern float bmp5xx_pressure;

/** Public functions. *********************************************************/

/**
 * @brief Initialize BMP5xx with BMP5 driver.
 *
 * @return Result of BMP5 API execution status.
 * @retval == 0 -> Success.
 * @retval > 0  -> Warning.
 * @retval < 0  -> Error.
 */
int8_t bmp5xx_init(void);

/**
 * @brief Update pressure and temperature data using moving average filtering.
 *
 * @return Result of BMP5 API execution status.
 * @retval == 0 -> Success.
 * @retval > 0  -> Warning.
 * @retval < 0  -> Error.
 */
int8_t bmp5xx_get_data(void);

#endif
