/*******************************************************************************
 * @file ublox_hal_uart.c
 * @brief u-blox NMEA protocol functions: abstracting STM32 HAL: UART.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "ublox_hal_uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** Definitions. **************************************************************/

#define GNGGA_TOKEN_COUNT 12

/** Private variables. ********************************************************/

// Buffer for UART reception.
static uint8_t ublox_rx_dma_buffer[UBLOX_RX_BUFFER_SIZE];

// Latest GPS data.
ublox_data_t gps_data = {0};

// Rx buffer management for DMA based operation.
static uint16_t ublox_rx_index = 0;
static bool ublox_in_sentence = false;
static uint8_t sentence_start_index = 0;
static uint8_t sentence_end_index = 0;

/** Private functions. ********************************************************/

/**
 * @brief Generic void error handler.
 */
void ublox_error_handler(void) {}

/**
 * @brief Convert latitude/longitude from DDMM.MMMM to decimal degrees.
 *
 * @param coordinate Original degrees and minutes measurements.
 * @param direction Original direction measurement (N, S, E, W).
 *
 * @return Converted decimal degrees measurement.
 */
double to_decimal_deg(const char *coordinate, const char direction) {
  // Parse the degrees and minutes.
  double value = strtod(coordinate, NULL);
  int degrees = (int)(value / 100);
  double minutes = value - (degrees * 100);

  // Convert to decimal degrees.
  double decimal_degrees = degrees + (minutes / 60.0);

  // Apply direction correction (negative for S or W).
  if (direction == 'S' || direction == 'W') {
    decimal_degrees = -decimal_degrees;
  }

  return decimal_degrees;
}

/**
 * @brief Compute the NMEA-style XOR checksum over a single character-buffer.
 *
 * This function expects to be passed exactly the characters between '$' and
 * the '*'. For example, if you want to checksum "$PUBX,41,1,0007,0003,9600,0",
 * call this with the pointer to "PUBX,41,1,0007,0003,9600,0" and the length
 * of that string.
 *
 * @param buf    Pointer to the first character AFTER the leading '$'.
 * @param length Length of the buffer to checksum.
 *
 * @return 8-bit XOR of all bytes in buf[0 .. length-1].
 */
static uint8_t compute_nmea_checksum(const char *buf, size_t length) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < length; i++) {
    checksum ^= (uint8_t)buf[i];
  }
  return checksum;
}

/**
 * @brief XOR of all chars between `$` and `*`, return parsed checksum.
 *
 * @param sentence NMEA sentence to process.
 *
 * @return bool
 * @retval == true -> Checksum pass.
 * @retval == false -> Checksum fail.
 */
static bool validate_nmea_checksum(const char *sentence) {
  const char *p = sentence;
  if (*p != '$')
    return false;
  uint8_t checksum = 0;
  while (*++p && *p != '*' && *p != '\r' && *p != '\n') {
    checksum ^= (uint8_t)*p;
  }
  if (*p != '*')
    return false;

  // Parse the two hex digits after '*'.
  char hexbuf[3] = {p[1], p[2], 0};
  char *endptr = NULL;
  uint8_t received = (uint8_t)strtoul(hexbuf, &endptr, 16);
  if (endptr != &hexbuf[2])
    return false;

  return checksum == received;
}

/** @brief Parse GNGGA fields.
 *
 * @return bool
 * @retval == true -> All values seem valid.
 * @retval == false -> At least 1 value seems invalid.
 *
 * @note GPS data is still updated on failure using information processed up to
 * (but not including) the invalid information.
 */
static bool parse_gngga(const char *sentence) {
  // Quick tokenization in-place.
  char buf[UBLOX_RX_BUFFER_SIZE];
  size_t len = strnlen(sentence, sizeof(buf) - 1);
  if (len >= sizeof(buf) - 1)
    return false;
  memcpy(buf, sentence, len);
  buf[len] = '\0';

  // Split into comma‑delimited tokens.
  char *tokens[GNGGA_TOKEN_COUNT] = {0};
  char *saveptr = NULL;
  tokens[0] = strtok_r(buf, ",", &saveptr);
  for (int i = 1; i < GNGGA_TOKEN_COUNT && tokens[i - 1]; ++i) {
    tokens[i] = strtok_r(NULL, ",", &saveptr);
  }
  // Must have at least fields up through geoid separation (index 11).
  if (!tokens[0] || !tokens[11])
    return false;

  // Field conversions.
  // UTC time.
  strncpy(gps_data.time, tokens[1], sizeof(gps_data.time) - 1);
  gps_data.time[sizeof(gps_data.time) - 1] = '\0';

  // Latitude and longitude.
  gps_data.latitude = to_decimal_deg(tokens[2], tokens[3][0]);
  gps_data.lon_dir = tokens[5][0];
  gps_data.longitude = to_decimal_deg(tokens[4], tokens[5][0]);
  gps_data.lat_dir = tokens[3][0];

  char *endptr = NULL; // Create null pointer for stdlib functions.

  // Fix quality.
  gps_data.fix_quality = (unsigned)strtoul(tokens[6], &endptr, 10);
  if (endptr == tokens[6])
    return false;

  // Number of satellites.
  gps_data.satellites = (unsigned)strtoul(tokens[7], &endptr, 10);
  if (endptr == tokens[7])
    return false;

  // hdop, altitude, geoid separation.
  gps_data.hdop = strtof(tokens[8], &endptr);
  gps_data.altitude = strtod(tokens[9], &endptr);
  gps_data.geoid_sep = strtod(tokens[11], &endptr);

  return true;
}

/**
 * @brief Top‑level sentence dispatcher.
 *
 * @param sentence NMEA sentence to process.
 */
static void parse_nmea_sentence(const char *sentence) {
  if (strncmp(sentence, "$GNGGA", 6) == 0) { // Handle GNGGA sentence.
    if (!validate_nmea_checksum(sentence) || !parse_gngga(sentence)) {
      ublox_error_handler();
    }
  }
  // Ignore other sentence types.
}

/**
 * @brief Process u-blox UART NEMA sentence byte.
 *
 * @param byte Byte value to process.
 * @param parse_index Current index in data buffer being processed.
 */
static void ublox_process_byte(uint8_t byte, size_t parse_index) {
  {
    if (!ublox_in_sentence && byte == '$') { // Start of new sentence.
      ublox_in_sentence = true;
      sentence_start_index = parse_index;
      return;
    }

    if (ublox_in_sentence && byte == '\n') { // Inside a sentence and found end.
      sentence_end_index = parse_index;

      // Extract the sentence [$ ... \n] into a contiguous buffer.
      static char sentence[UBLOX_RX_BUFFER_SIZE];
      uint16_t len;
      if (sentence_end_index >= sentence_start_index) { // Linear sentence.
        len = sentence_end_index - sentence_start_index + 1;
        memcpy(sentence, &ublox_rx_dma_buffer[sentence_start_index], len);
      } else { // Handle DMA wraparound overwrite.
        len = UBLOX_RX_BUFFER_SIZE - sentence_start_index;
        memcpy(sentence, &ublox_rx_dma_buffer[sentence_start_index], len);
        memcpy(sentence + len, ublox_rx_dma_buffer, sentence_end_index + 1);
        len += sentence_end_index + 1;
      }
      sentence[len] = '\0'; // Null‑terminate.

      parse_nmea_sentence(sentence); // Parse the extracted sentence.

      // Reset for next sentence.
      ublox_in_sentence = false;
      sentence_start_index = 0;
      sentence_end_index = 0;
    }
  }
}

/** User implementations of STM32 UART HAL (overwriting HAL). *****************/

void HAL_UART_RxCpltCallback_ublox(UART_HandleTypeDef *huart) {
  if (huart == &UBLOX_HUART) {
    for (size_t i = 0; i < UBLOX_RX_BUFFER_SIZE; ++i) {
      ublox_process_byte(ublox_rx_dma_buffer[i], i);
    }
  }
}

/** NOTE: USART2 hardware specific, implement in USART2_IRQHandler(). */
void USART2_IRQHandler_ublox(UART_HandleTypeDef *huart) {
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) { // Detected IDLE flag.
    __HAL_UART_CLEAR_IDLEFLAG(huart);               // Clear the IDLE flag.

    // Check how many bytes have been written by DMA since last time.
    uint16_t pos = UBLOX_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    // Process every new byte in order.
    while (ublox_rx_index != pos) {
      uint8_t b = ublox_rx_dma_buffer[ublox_rx_index];
      ublox_process_byte(b, ublox_rx_index);
      ublox_rx_index = (ublox_rx_index + 1) % UBLOX_RX_BUFFER_SIZE;
    }
  }
}

/** Public functions. *********************************************************/

void ublox_init(void) {
  // Ensure the u-blox module is not in reset state.
  HAL_GPIO_WritePin(UBLOX_RESETN_PORT, UBLOX_RESETN_PIN, GPIO_PIN_SET);

  // Swap to a higher baud rate on the u-blox module, then on the STM32.
  ublox_set_baudrate(UBLOX_INIT_BAUD_RATE);
  UBLOX_HUART.Init.BaudRate = UBLOX_INIT_BAUD_RATE;
  HAL_UART_Init(&UBLOX_HUART);

  // Start UART reception with DMA.
  HAL_UART_Receive_DMA(&UBLOX_HUART, ublox_rx_dma_buffer, UBLOX_RX_BUFFER_SIZE);
}

void ublox_reset(void) {
  HAL_GPIO_WritePin(UBLOX_RESETN_PORT, UBLOX_RESETN_PIN, GPIO_PIN_RESET);
  HAL_Delay(5); // Hold reset for 5 ms.
  HAL_GPIO_WritePin(UBLOX_RESETN_PORT, UBLOX_RESETN_PIN, GPIO_PIN_SET);
}

void ublox_set_baudrate(uint32_t baudrate) {
  // 1) Build the core body of the PUBX,41 sentence (no '$' and no "*CS"),
  //    e.g. "PUBX,41,1,0007,0003,9600,0".
  char core_msg[64];
  int core_len = snprintf(core_msg, sizeof(core_msg),
                          "PUBX,41,1,0007,0003,%lu,0", (unsigned long)baudrate);
  if (core_len < 0 || core_len >= (int)sizeof(core_msg)) {
    // snprintf error or truncation (should not happen with 64 bytes).
    return;
  }

  // 2) Compute the XOR checksum over core_msg (i.e. the bytes after '$').
  uint8_t cs = compute_nmea_checksum(core_msg, (size_t)core_len);

  // 3) Build the final NMEA string:
  //    "$" + core_msg + "*" + two-digit hex CS + "\r\n".
  //    Example result: "$PUBX,41,1,0007,0003,9600,0*2A\r\n".
  char full_sentence[80];
  int full_len = snprintf(full_sentence, sizeof(full_sentence), "$%s*%02X\r\n",
                          core_msg, cs);
  if (full_len < 0 || full_len >= (int)sizeof(full_sentence)) {
    // snprintf error (should not happen with 80 bytes)
    return;
  }

  // 4) Transmit it out over the same UART that feeds the SAM-M10Q.
  //    This will immediately reconfigure the module's UART port to new_baud.
  HAL_UART_Transmit(&UBLOX_HUART, (uint8_t *)full_sentence, (uint16_t)full_len,
                    HAL_MAX_DELAY);

  // NOTE: After sending this string, the SAM-M10Q's UART hardware switches
  //       to `new_baud`. If you want to continue communicating at new_baud,
  //       you must reconfigure huartx.Init.BaudRate = new_baud and call
  //       HAL_UART_Init(&huartx) on the STM32 side.
}
