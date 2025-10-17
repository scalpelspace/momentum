/*******************************************************************************
 * @file    comm.h
 * @brief   USART1 Communication Interface: abstracting STM32 HAL: UART.
 *******************************************************************************/

#ifndef MOMENTUM__COMM_H
#define MOMENTUM__COMM_H

/** Includes. *****************************************************************/

#include "stm32l4xx_hal.h"
#include <stdbool.h>

/** STM32 port and pin configs. ***********************************************/

extern UART_HandleTypeDef huart1;

// UART.
#define COMM_HUART huart1

/** Definitions. **************************************************************/

#define FRAME_START 0x7E

/** User implementations of STM32 NVIC HAL (overwriting HAL). *****************/

void HAL_UART_RxCpltCallback_comm(UART_HandleTypeDef *huart);
void USART1_IRQHandler_comm(UART_HandleTypeDef *huart);

/** Public functions. *********************************************************/

void comm_init(void);
void comm_process_rx_data(void);
void comm_send_packet(uint8_t command, const uint8_t *payload, uint16_t len);

#endif
