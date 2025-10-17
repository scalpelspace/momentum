/*******************************************************************************
 * @file comm.c
 * @brief USART1 Communication Interface: abstracting STM32 HAL: UART.
*******************************************************************************
 * @note:
 * USART1 (UART1) communication must be enabled via the `configuration.h` macro:
 * `MOMENTUM_COMM_ENABLE`. Transmit functions should be handled via `stdio.h`
 * `printf()` function. The `_write()` function is implemented in `comm.c`.
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

/** Private variables. ********************************************************/

static uint8_t comm_rx_dma_buffer[COMM_RX_BUFFER_SIZE];

static uint8_t comm_rx_dma_length;  // DMA new Rx data length.
static uint8_t comm_rx_dma_old_pos; // DMA new Rx data buffer index.

/** Private functions. ********************************************************/

int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&COMM_HUART, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}

/** User implementations of STM32 UART HAL (overwriting HAL). *****************/

void HAL_UART_RxCpltCallback_comm(UART_HandleTypeDef *huart) {
  if (huart == &COMM_HUART && !comm_rx_dma_length) { // Ready for new data.
    comm_rx_dma_length =
        (uint8_t)COMM_RX_BUFFER_SIZE; // Update DMA new Rx data length.
  }
}

/** NOTE: USART1 hardware specific, implement in USART1_IRQHandler(). */
void USART1_IRQHandler_comm(UART_HandleTypeDef *huart) {
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) { // Detected IDLE flag.
    __HAL_UART_CLEAR_IDLEFLAG(huart);               // Clear the IDLE flag.

    // Total number of bytes the DMA has written into the buffer.
    uint16_t pos = COMM_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    if (!comm_rx_dma_length && pos != comm_rx_dma_old_pos) {
      // Ready for new data and at least 1 new byte received.
      comm_rx_dma_length = (COMM_RX_BUFFER_SIZE - comm_rx_dma_old_pos) +
                           pos; // Update DMA new Rx data length.
    }
    return;
  }
}

/** Public functions. *********************************************************/

void comm_init(void) {
  comm_rx_dma_length = 0;  // Reset DMA new Rx data length.
  comm_rx_dma_old_pos = 0; // Reset DMA new Rx data buffer index.

  // Start UART reception with DMA and enable IDLE based flagging.
  HAL_UART_Receive_DMA(&COMM_HUART, comm_rx_dma_buffer, COMM_RX_BUFFER_SIZE);
  __HAL_UART_ENABLE_IT(&COMM_HUART, UART_IT_IDLE);
}

void comm_process_rx_data(void) {
  if (comm_rx_dma_length == 0) {
    return;
  }

  // 1) The new data fits entirely before the end of buffer.
  if (comm_rx_dma_old_pos + comm_rx_dma_length <= COMM_RX_BUFFER_SIZE) {

    // TODO: Example RX buffer handler.
    // process(&comm_rx_dma_buffer[comm_rx_dma_old_pos], comm_rx_dma_length);
    // comm_rx_dma_old_pos += comm_rx_dma_length;

    // Wrap exactly at the boundary.
    if (comm_rx_dma_old_pos == COMM_RX_BUFFER_SIZE) {
      comm_rx_dma_old_pos = 0;
    }
  } else {
    // 2) Overrun the end, use two slices.

    // First slice: from old_pos to buffer end.
    // TODO: Example RX buffer handler.
    // uint16_t first_len = COMM_RX_BUFFER_SIZE - comm_rx_dma_old_pos;
    // process(&comm_rx_dma_buffer[comm_rx_dma_old_pos], first_len);

    // Second slice: from buffer start for the remainder.
    // TODO: Example RX buffer handler.
    // uint16_t second_len = comm_rx_dma_length - first_len;
    // process(&comm_rx_dma_buffer[0], second_len);
    // comm_rx_dma_old_pos = second_len; // Wrapped start index.
  }

  // 3) Clear the "new data" flag.
  comm_rx_dma_length = 0;
}
