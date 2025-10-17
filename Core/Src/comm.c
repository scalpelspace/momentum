/*******************************************************************************
 * @file comm.c
 * @brief USART1 Communication Interface: abstracting STM32 HAL: UART.
 *******************************************************************************
 * @note:
 * Transmit functions should be handled via `stdio.h` `printf()` function. The
 * `_write()` function is implemented in `comm.c`.
 *******************************************************************************
 */

/** Definitions. **************************************************************/

#include "comm.h"
#include "ws2812b_hal_pwm.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/** Definitions. **************************************************************/

#define COMM_TX_BUFFER_SIZE 128
#define COMM_RX_BUFFER_SIZE 128

/** Private functions. ********************************************************/

int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&COMM_HUART, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}
