/*******************************************************************************
 * @file    comm.c
 * @brief   USART1 Communication Interface: abstracting STM32 HAL: UART.
 *******************************************************************************
 */

/** Definitions. **************************************************************/

#include "comm.h"
#include "logger.h"
#include "w25qxx_hal_spi.h"
#include "ws2812b_hal_pwm.h"
#include <stdint.h>
#include <string.h>

/** Definitions. **************************************************************/

#define COMM_READ_MAX_CHUNK_SIZE 256
#define MAX_FRAME_LEN 272 // 1 + 1 + 2 + COMM_READ_MAX_CHUNK_SIZE + 2.

#define COMM_RX_BUFFER_SIZE 256

/** Private variables. ********************************************************/

static uint8_t comm_rx_dma_buffer[COMM_RX_BUFFER_SIZE];

static uint8_t comm_rx_dma_length;  // DMA new Rx data length.
static uint8_t comm_rx_dma_old_pos; // DMA new Rx data buffer index.

static uint8_t read_page_buf[COMM_READ_MAX_CHUNK_SIZE];

/** Private functions. ********************************************************/

static uint16_t crc16_calc(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;        // Initial value.
  const uint16_t poly = 0x1021; // CRC-16-CCITT polynomial.
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ poly;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

static bool crc16_check(const uint8_t *frame, uint16_t frame_len) {
  if (frame_len < 2)
    return false;
  uint16_t received = (frame[frame_len - 2] << 8) | frame[frame_len - 1];
  uint16_t calculated = crc16_calc(frame, frame_len - 2);
  return (received == calculated);
}

static void send_packet(uint8_t command, const uint8_t *payload,
                        const uint16_t len) {
  uint8_t comm_tx_dma_buffer[MAX_FRAME_LEN];

  uint16_t idx = 0;
  comm_tx_dma_buffer[idx++] = FRAME_START;
  comm_tx_dma_buffer[idx++] = command;
  comm_tx_dma_buffer[idx++] = (len >> 8) & 0xFF;
  comm_tx_dma_buffer[idx++] = len & 0xFF;
  if (len) {
    memcpy(&comm_tx_dma_buffer[idx], payload, len);
    idx += len;
  }
  uint16_t crc = crc16_calc(comm_tx_dma_buffer, idx);
  comm_tx_dma_buffer[idx++] = (crc >> 8) & 0xFF;
  comm_tx_dma_buffer[idx++] = crc & 0xFF;
  HAL_UART_Transmit(&COMM_HUART, comm_tx_dma_buffer, idx, HAL_MAX_DELAY);
}

static void process_frame(uint8_t *frame, uint8_t length) {
  uint8_t command = frame[1];
  uint16_t payload_len = (frame[2] << 8) | frame[3];
  uint8_t *payload = &frame[4];

  switch (command) {

  case CMD_NVM_RESET:
    ws2812b_set_colour(0, 3, 1, 0); // Wipe process colour.
    ws2812b_update();
    logger_hard_reset();
    send_packet(CMD_ACK, &command, 1);
    ws2812b_set_colour(0, 2, 1, 3); // Wipe success colour.
    ws2812b_update();
    break;

  case CMD_NVM_WRITE:
    // payload = [ADDR_H][ADDR_M][ADDR_L][DATA...].
    uint32_t write_addr = (payload[0] << 16) | (payload[1] << 8) | payload[2];
    uint8_t *data = &payload[3];
    uint16_t write_len = payload_len - 3;
    if (w25q_page_program(data, write_addr, write_len) == HAL_OK) {
      send_packet(CMD_ACK, &command, 1);
    } else {
      send_packet(CMD_NACK, &command, 1);
    }
    break;

  case CMD_NVM_READ:
    // payload = [ADDR_H][ADDR_M][ADDR_L][LEN_H][LEN_L].
    uint32_t read_addr = (payload[0] << 16) | (payload[1] << 8) | payload[2];
    uint16_t read_len = (payload[3] << 8) | payload[4];
    if (read_len > COMM_READ_MAX_CHUNK_SIZE) { // Limit to max read chunk size.
      send_packet(CMD_NACK, &command, 1);
      break;
    }
    if (w25q_read_data(read_page_buf, read_addr, read_len) == HAL_OK) {
      send_packet(CMD_DATA, read_page_buf, read_len);
    } else {
      send_packet(CMD_NACK, &command, 1);
    }
    break;

  default: // Unknown command.
    send_packet(CMD_NACK, &command, 1);
    break;
  }
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
    process_frame(&comm_rx_dma_buffer[comm_rx_dma_old_pos], comm_rx_dma_length);

    comm_rx_dma_old_pos += comm_rx_dma_length;
    // Wrap exactly at the boundary.
    if (comm_rx_dma_old_pos == COMM_RX_BUFFER_SIZE) {
      comm_rx_dma_old_pos = 0;
    }
  } else {
    // 2) Overrun the end, use two slices.
    // First slice: from old_pos to buffer end.
    uint16_t first_len = COMM_RX_BUFFER_SIZE - comm_rx_dma_old_pos;
    process_frame(&comm_rx_dma_buffer[comm_rx_dma_old_pos], first_len);

    // Second slice: from buffer start for the remainder.
    uint16_t second_len = comm_rx_dma_length - first_len;
    process_frame(&comm_rx_dma_buffer[0], second_len);

    comm_rx_dma_old_pos = second_len; // Wrapped start index.
  }

  // 3) Clear the "new data" flag.
  comm_rx_dma_length = 0;
}
