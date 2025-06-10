/*******************************************************************************
 * @file    comm.c
 * @brief   USART1 Communication Interface: abstracting STM32 HAL: UART.
 *******************************************************************************
 */

/** Definitions. **************************************************************/

#include "comm.h"
#include "w25qxx_hal_spi.h"
#include <string.h>

/** Definitions. **************************************************************/

#define MAX_FRAME_LEN 272 // 1 + 1 + 2 + 256 + 2.

#define COMM_RX_BUFFER_SIZE 256
#define READ_CHUNK_SIZE 256

/** Private variables. ********************************************************/

static uint8_t comm_rx_dma_buffer[COMM_RX_BUFFER_SIZE];
static uint8_t comm_tx_dma_buffer[MAX_FRAME_LEN];

static bool write_enabled = false;
static uint8_t read_page_buf[READ_CHUNK_SIZE];

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

static void send_packet(uint8_t command, uint8_t *payload, uint16_t len) {
  uint16_t idx = 0;
  comm_tx_dma_buffer[idx++] = FRAME_START;
  comm_tx_dma_buffer[idx++] = command;
  comm_tx_dma_buffer[idx++] = (len >> 8) & 0xFF;
  comm_tx_dma_buffer[idx++] = len & 0xFF;
  if (payload && len) {
    memcpy(&comm_tx_dma_buffer[idx], payload, len);
    idx += len;
  }
  uint16_t crc = crc16_calc(comm_tx_dma_buffer, idx);
  comm_tx_dma_buffer[idx++] = (crc >> 8) & 0xFF;
  comm_tx_dma_buffer[idx++] = crc & 0xFF;
  HAL_UART_Transmit_DMA(&COMM_HUART, comm_tx_dma_buffer, idx);
}

/** User implementations of STM32 UART HAL (overwriting HAL). *****************/

void HAL_UART_RxCpltCallback_comm(UART_HandleTypeDef *huart) {
  if (huart == &COMM_HUART) {
    comm_process_frame(comm_rx_dma_buffer, COMM_RX_BUFFER_SIZE);
  }
}

/** NOTE: USART1 hardware specific, implement in USART1_IRQHandler(). */
void USART1_IRQHandler_comm(UART_HandleTypeDef *huart) {
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) { // Detected IDLE flag.
    __HAL_UART_CLEAR_IDLEFLAG(huart);               // Clear the IDLE flag.

    // Check how many bytes have been written by DMA since last time.
    uint16_t bytes = COMM_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    // Process new packet.
    comm_process_frame(comm_rx_dma_buffer, bytes);

    // Rearm DMA receive.
    HAL_UART_Receive_DMA(huart, comm_rx_dma_buffer, COMM_RX_BUFFER_SIZE);
  }
}

/** Public functions. *********************************************************/

void comm_init(void) {
  write_enabled = false;

  // Start UART reception with DMA and enable IDLE based flagging.
  HAL_UART_Receive_DMA(&COMM_HUART, comm_rx_dma_buffer, COMM_RX_BUFFER_SIZE);
  __HAL_UART_ENABLE_IT(&COMM_HUART, UART_IT_IDLE);
}

void comm_process_frame(uint8_t *frame, uint16_t len) {
  uint8_t command = frame[1];
  uint16_t payload_len = (frame[2] << 8) | frame[3];
  uint8_t *payload = &frame[4];

  switch (command) {
  case CMD_WRITE_EN:
    write_enabled = true;
    send_packet(CMD_ACK, &command, 1);
    break;

  case CMD_WRITE_DEN:
    write_enabled = false;
    send_packet(CMD_ACK, &command, 1);
    break;

  case CMD_WRITE:
    if (!write_enabled) {
      send_packet(CMD_NACK, &command, 1);
      break;
    } else {
      // payload: [ADDR_H][ADDR_M][ADDR_L][DATA...].
      uint32_t write_addr = (payload[0] << 16) | (payload[1] << 8) | payload[2];
      uint8_t *data = &payload[3];
      uint16_t write_len = payload_len - 3;
      if (w25q_page_program(data, write_addr, write_len) == HAL_OK) {
        send_packet(CMD_ACK, &command, 1);
      } else {
        send_packet(CMD_NACK, &command, 1);
      }
    }
    break;

  case CMD_READ_DATA:
    // payload: [ADDR_H][ADDR_M][ADDR_L][LEN_H][LEN_L].
    uint32_t read_addr = (payload[0] << 16) | (payload[1] << 8) | payload[2];
    uint16_t rlen = (payload[3] << 8) | payload[4];
    if (rlen > 256) // Clamp to max chunk size.
      rlen = 256;
    if (w25q_read_data(read_page_buf, read_addr, rlen) == HAL_OK) {
      send_packet(CMD_DATA, read_page_buf, rlen);
    } else {
      send_packet(CMD_NACK, &command, 1);
    }
    break;

  default: // Unknown command.
    send_packet(CMD_NACK, &command, 1);
    break;
  }
}
