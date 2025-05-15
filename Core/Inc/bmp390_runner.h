/*******************************************************************************
 * @file bmp390_runner.h
 * @brief BMP390 runner: init and data reading.
 *******************************************************************************
 */

#ifndef MOMENTUM__BMP390_RUNNER_H
#define MOMENTUM__BMP390_RUNNER_H

/** Includes. *****************************************************************/

#include "bmp3.h"
#include "bmp3_hal_i2c.h"

/** Definitions. **************************************************************/

// Defines frame count to be requested.
#define FIFO_FRAME_COUNT UINT8_C(64)

#define FIFO_MAX_SIZE UINT16_C(512) // BMP390 Maximum FIFO size.
// Max is 512 total.
// Technically the FIFO_MAX_SIZE can be reduced to FIFO_FRAME_COUNT * 8.
// Header:      2 bytes.
// Temperature: 3 bytes.
// Pressure:    3 bytes.

/** Public variables. *********************************************************/

extern double bmp390_temperature;
extern double bmp390_pressure;

/** Public functions. *********************************************************/

/**
 * @brief Initialize BMP390 with BMP3 driver.
 *
 * @return Result of BMP3 API execution status.
 * @retval == 0 -> Success.
 * @retval > 0  -> Warning.
 * @retval < 0  -> Error.
 */
int8_t bmp390_init(void);

/**
 * @brief Update pressure and temperature data using moving average filtering.
 *
 * @return Result of BMP3 API execution status.
 * @retval == 0 -> Success.
 * @retval > 0  -> Warning.
 * @retval < 0  -> Error.
 */
void bmp390_get_data(void);

#endif
