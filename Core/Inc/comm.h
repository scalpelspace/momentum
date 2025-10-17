/*******************************************************************************
 * @file comm.h
 * @brief USART1 Communication Interface: abstracting STM32 HAL: UART.
 *******************************************************************************
 * @note:
 * USART1 (UART1) communication must be enabled via the `configuration.h` macro:
 * `MOMENTUM_COMM_ENABLE`. Transmit functions should be handled via `stdio.h`
 * `printf()` function. The `_write()` function is implemented in `comm.c`.
 *******************************************************************************
 */

#ifndef MOMENTUM__COMM_H
#define MOMENTUM__COMM_H

/** Includes. *****************************************************************/

#include "stm32l4xx_hal.h"

/** STM32 port and pin configs. ***********************************************/

extern UART_HandleTypeDef huart1;

// UART.
#define COMM_HUART huart1

/** User implementations of STM32 NVIC HAL (overwriting HAL). *****************/

void HAL_UART_RxCpltCallback_comm(UART_HandleTypeDef *huart);
void USART1_IRQHandler_comm(UART_HandleTypeDef *huart);

/** Public functions. *********************************************************/

void comm_init(void);
void comm_process_rx_data(void);

#endif
