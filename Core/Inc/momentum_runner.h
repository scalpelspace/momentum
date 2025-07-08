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

/** Definitions. **************************************************************/

// Software version: `MAJOR.MINOR.PATCH-IDENTIFIER`.
#define MOMENTUM_VERSION_MAJOR 0        // uint8_t number.
#define MOMENTUM_VERSION_MINOR 1        // uint8_t number.
#define MOMENTUM_VERSION_PATCH 0        // uint8_t number.
#define MOMENTUM_VERSION_IDENTIFIER 'a' // ASCII character.

/** STM32 port and pin configs. ***********************************************/

extern SPI_HandleTypeDef hspi3;

// SPI.
#define MOMENTUM_HSPI hspi3
// CS pin is handled by hardware NSS.

/** User implementations of STM32 UART HAL (overwriting HAL). *****************/

void HAL_SPI_RxCpltCallback_momentum(SPI_HandleTypeDef *hspi);

void HAL_SPI_TxCpltCallback_momentum(SPI_HandleTypeDef *hspi);

/** Public functions. *********************************************************/

void momentum_spi_start(void);

#endif
