/*******************************************************************************
 * @file callbacks.c
 * @brief STM32 HAL callback implementations overriding weak declarations.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "comm.h"
#include "configuration.h"
#include "momentum_runner.h"
#include "sh2_hal_spi.h"
#include "stm32l4xx_hal.h"
#include "ublox_hal_uart.h"

/** Collection of user implementations of STM32 HAL (overwriting HAL). ********/

/** GPIO. */

void HAL_GPIO_EXTI_Callback(uint16_t n) { HAL_GPIO_EXTI_Callback_sh2(n); }

/** UART. */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
#ifdef MOMENTUM_COMM_ENABLE
  HAL_UART_RxCpltCallback_comm(huart);
#endif
  HAL_UART_RxCpltCallback_ublox(huart);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {}

/** SPI. */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  HAL_SPI_TxRxCpltCallback_sh2(hspi);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
#ifndef MOMENTUM_W25QXX_ENABLE
  HAL_SPI_RxCpltCallback_momentum(hspi);
#endif
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
#ifndef MOMENTUM_W25QXX_ENABLE
  HAL_SPI_TxCpltCallback_momentum(hspi);
#endif
}
