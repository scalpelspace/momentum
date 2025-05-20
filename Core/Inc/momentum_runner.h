/*******************************************************************************
 * @file momentum_runner.h
 * @brief Momentum runner handling low level interacting functions.
 *******************************************************************************
 */

#ifndef MOMENTUM__RUNNER_H
#define MOMENTUM__RUNNER_H

/** Includes. *****************************************************************/

#include "momentum_driver.h"
#include "stm32l4xx_hal.h"

/** STM32 port and pin configs. ***********************************************/

extern SPI_HandleTypeDef hspi3;

// SPI.
#define MOMENTUM_HSPI hspi3
// CS pin is handled by hardware NSS.

/** Public functions. *********************************************************/

void HAL_SPI_TxRxCpltCallback_momentum(SPI_HandleTypeDef *hspi);

void momentum_spi_start(void);

#endif
