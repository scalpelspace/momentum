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

#define CMD_ACK 0x06
#define CMD_NACK 0x07

// NVM control.
#define CMD_WRITE_EN 0x10
#define CMD_WRITE_DEN 0x11
#define CMD_WRITE 0x12
#define CMD_READ_DATA 0x20
#define CMD_DATA 0x21

/** Public variables. *********************************************************/

extern bool comm_write_enabled;

/** User implementations of STM32 NVIC HAL (overwriting HAL). *****************/

void HAL_UART_RxCpltCallback_comm(UART_HandleTypeDef *huart);
void USART1_IRQHandler_comm(UART_HandleTypeDef *huart);

/** Public functions. *********************************************************/

void comm_init(void);
void comm_process_rx_data(void);

#endif
