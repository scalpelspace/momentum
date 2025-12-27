/*******************************************************************************
 * @file comm.c
 * @brief USART1 Communication Interface: abstracting STM32 HAL: UART.
 *******************************************************************************
 * @note:
 * Transmit functions should be handled via `stdio.h` `printf()` function. The
 * `_write()` function is implemented in `comm.c`.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "comm.h"
#include "ws2812b_hal_pwm.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/** Definitions. **************************************************************/

#define FW_VERSION_STR "v0.1.0-p"

#define COMM_LINE_MAX 96

/** Private variables. ********************************************************/

static char comm_line[COMM_LINE_MAX];
static volatile uint16_t comm_line_len = 0;
static uint8_t comm_rx_byte;

/** Private functions. ********************************************************/

int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&COMM_HUART, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}

static void comm_handle_line(const char *line) {
  // Skip leading spaces.
  while (*line == ' ' || *line == '\t')
    line++;

  if (strcmp(line, "version") == 0 || strcmp(line, "ver") == 0) {
    printf("FW %s\r\n", FW_VERSION_STR);
    return;
  }

  if (*line) {
    // Error: unknown command.
  }
}

/** User implementations into STM32 HAL (overwrite weak HAL functions). *******/

void HAL_UART_RxCpltCallback_comm(UART_HandleTypeDef *huart) {
  if (huart == &COMM_HUART) {
    uint8_t b = comm_rx_byte;

    HAL_UART_Receive_DMA(&COMM_HUART, &comm_rx_byte, 1);

    // Basic line editing/framing.
    if (b == '\r') {
      return; // Ignore CR.
    }

    if (b == '\n') {
      // End of line: null-terminate and handle command.
      if (comm_line_len >= COMM_LINE_MAX)
        comm_line_len = COMM_LINE_MAX - 1;
      comm_line[comm_line_len] = '\0';

      comm_handle_line(comm_line);

      // Reset buffer.
      comm_line_len = 0;
      return;
    }

    // Normal character.
    if (comm_line_len < (COMM_LINE_MAX - 1)) {
      comm_line[comm_line_len++] = (char)b;
    } else {
      // Overflow: drop line.
      comm_line_len = 0;
    }
  }
}

/** Public functions. *********************************************************/

void comm_init(void) {
  comm_line_len = 0;
  // Initialize single byte DMA.
  HAL_UART_Receive_DMA(&COMM_HUART, &comm_rx_byte, 1);
}
