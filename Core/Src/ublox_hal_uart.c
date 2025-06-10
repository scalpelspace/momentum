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

#define GNGGA_TOKEN_COUNT 15 // GGA index [0..14], exclude checksum and return.
#define GNRMC_TOKEN_COUNT 14 // RMC index [0..13], exclude checksum and return.

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
 * @brief Given the three NMEA fix‐flags (status, quality, pos_mode), determine
 *        which of the 10 possible nmea_position_fix_t types.
 *
 * @param flags Pointer to the three‐field struct filled from NMEA sentences:
 *              - flags->status  = 'A' or 'V'.
 *              - flags->quality = 0,1,2,4,5,6.
 *              - flags->pos_mode = 'N','A','D','E','R','F'.
 *
 * @return One of the 10 position_fix_t enum values.
 */
nmea_position_fix_t classify_position_fix(const nmea_fix_flags_t *flags) {
  // 1) No position fix at power-up or after losing satellite lock.
  //    (status='V', quality=0, pos_mode='N').
  if (flags->status == 'V' && flags->quality == 0 && flags->pos_mode == 'N') {
    return FIX_TYPE_NO_FIX;
  }

  // 2) GNSS fix, but user limits exceeded.
  //    (status='V', quality=0, pos_mode='A' or 'D').
  if (flags->status == 'V' && flags->quality == 0 &&
      (flags->pos_mode == 'A' || flags->pos_mode == 'D')) {
    return FIX_TYPE_GNSS_LIMITS_EXCEEDED;
  }

  // 3) Dead-reckoning fix, but user limits exceeded.
  //    (status='V', quality=6, pos_mode='E').
  if (flags->status == 'V' && flags->quality == 6 && flags->pos_mode == 'E') {
    return FIX_TYPE_DR_LIMITS_EXCEEDED;
  }

  // 4) Dead-reckoning fix.
  //    (status='A', quality=6, pos_mode='E').
  if (flags->status == 'A' && flags->quality == 6 && flags->pos_mode == 'E') {
    return FIX_TYPE_DR_FIX;
  }

  // 5) RTK float.
  //    (status='A', quality=5, pos_mode='F').
  if (flags->status == 'A' && flags->quality == 5 && flags->pos_mode == 'F') {
    return FIX_TYPE_RTK_FLOAT;
  }

  // 6) RTK fixed.
  //    (status='A', quality=4, pos_mode='R').
  if (flags->status == 'A' && flags->quality == 4 && flags->pos_mode == 'R') {
    return FIX_TYPE_RTK_FIX;
  }

  // 7) GNSS 2D, GNSS 3D or GNSS + DR combined fix.
  //    (status='A', quality=1 or 2, pos_mode = 'A' or 'D').
  if (flags->status == 'A' && (flags->quality == 1 || flags->quality == 2) &&
      (flags->pos_mode == 'A' || flags->pos_mode == 'D')) {
    return FIX_TYPE_GNSS_FIX;
  }

  // 9) If none of the above matched, fall back to FIX_TYPE_UNDETERMINED.
  return FIX_TYPE_UNDETERMINED;
}

/**
 * @brief Convert latitude/longitude from DDMM.MMMM to decimal degrees.
 *
 * @param coordinate Original degrees and minutes measurements.
 * @param direction Original direction measurement (N, S, E, W).
 *
 * @return Converted decimal degrees measurement.
 */
float to_decimal_deg(const char *coordinate, const char direction) {
  // Parse the degrees and minutes.
  float value = strtof(coordinate, NULL);
  int degrees = (int)(value / 100);
  float minutes = value - (float)(degrees * 100);

  // Convert to decimal degrees.
  float decimal_degrees = (float)degrees + (minutes / 60.0f);

  // Apply direction correction (negative for S or W).
  if (direction == 'S' || direction == 'W') {
    decimal_degrees = -decimal_degrees;
  }

  return decimal_degrees;
}

/**
 * @brief Compute the two‐byte UBX checksum (Fletcher-like) over a byte buffer.
 *
 * This function implements the standard u-blox UBX checksum algorithm:
 *   - CK_A is the 8-bit sum of all bytes in the buffer.
 *   - CK_B is the 8-bit sum of all intermediate CK_A values.
 * Use this to checksum the "class, id, length, payload" portion of any UBX
 * packet before appending CK_A and CK_B to the end.
 *
 * @param buf Pointer to the start of the byte buffer to checksum (typically
 *            the message's Class, ID, Length, and Payload fields).
 * @param length Number of bytes in buf to include in the checksum calculation.
 * @param ck_a Pointer to an 8-bit variable to update to the computed CK_A.
 * @param ck_b Pointer to an 8-bit variable to update to the computed CK_B.
 */
static void compute_ubx_checksum(const uint8_t *buf, uint16_t length,
                                 uint8_t *ck_a, uint8_t *ck_b) {
  uint8_t a = 0, b = 0;
  for (uint16_t i = 0; i < length; i++) {
    a = a + buf[i];
    b = b + a;
  }
  *ck_a = a;
  *ck_b = b;
}

/**
 * @brief Compute the NMEA-style XOR checksum over a single character-buffer.
 *
 * This function expects to be passed exactly the characters between '$' and
 * the '*'. For example, if you want to checksum "$PUBX,41,1,0007,0003,9600,0",
 * call this with the pointer to "PUBX,41,1,0007,0003,9600,0" and the length
 * of that string.
 *
 * @param buf Pointer to the first character AFTER the leading '$'.
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

/**
 * @brief Split an NMEA sentence (comma‐delimited) into exactly N tokens,
 *        preserving empty (zero‐length) fields.  Tokens are placed in the
 *        pre‐allocated array `tokens[]`, and each token is a pointer into the
 *        single buffer `sentence`.  After calling this, buf[] will be modified
 *        in place (each comma becomes `\0`).
 *
 * @param sentence The NMEA line (null‐terminated); this buffer is modified.
 * @param tokens Array of pointers, length = max_tokens.
 * @param max_tokens Maximum number of tokens to extract.
 *
 * @return Number of tokens actually found (could be <= max_tokens).
 */
static int nmea_split_preserve_empty(char *sentence, char *tokens[],
                                     int max_tokens) {
  int count = 0;
  char *p = sentence;

  // The first token always starts at sentence (even if sentence[0] == ',').
  tokens[count++] = p;

  while (count < max_tokens && *p != '\0') {
    if (*p == ',') {
      // Replace comma with NUL, and make next token point to p + 1.
      *p = '\0';
      tokens[count++] = p + 1;
    }
    p++;
  }

  // If the very last character was a comma, it creates one more empty field
  // beyond what the loop saw.  E.g. "A,B," -> 3 tokens: "A", "B", "". But
  // because we only increment count when we see a comma inside the loop, if
  // the sentence ends with a comma we will still need to account for that:
  if (count < max_tokens && p > sentence && *(p - 1) == ',') {
    // sentence ended with a comma, so there is an empty token at the end.
    tokens[count++] = p; // p points to '\0', so this token is an empty string.
  }

  return count;
}

/** @brief Parse GNGGA fields.
 *
 * @param sentence Pointer to a null-terminated NMEA sentence string.
 *
 * @return bool
 * @retval == true -> All values seem valid.
 * @retval == false -> At least 1 value seems invalid.
 *
 * @note GPS data is still updated on failure using information processed up to
 *       (but not including) the invalid information.
 *
 *  tokens[0] = xxGGA         (string).
 *  tokens[1] = UTC time      (hhmmss.ss).
 *  tokens[2] = latitude      (ddmm.mmmmm).
 *  tokens[3] = N/S indicator ('N' or 'S').
 *  tokens[4] = longitude     (dddmm.mmmmm).
 *  tokens[5] = E/W indicator ('E' or 'W').
 *  tokens[6] = quality       (0..6) (empty string means missing).
 *  tokens[7] = numSat        (1..12+, as string).
 *  tokens[8] = HDOP          (float string).
 *  tokens[9] = altitude      (float string).
 * tokens[10] = altUnit       (unit, should be 'M').
 * tokens[11] = geoidSep      (float string).
 * tokens[12] = geoidSepUnit  (unit, should be 'M').
 * tokens[13] = diffAge       (character).
 * tokens[14] = diffStation   (ID of station providing differential correction).
 * tokens[15] = checksum      (hexadecimal string with leading '*').
 * tokens[16] = CRLF          (character).
 */
static bool parse_gngga(const char *sentence) {
  // 1) Copy into a local buffer for strtok_r.
  char buf[UBLOX_RX_BUFFER_SIZE];
  size_t len = strnlen(sentence, sizeof(buf) - 1);
  if (len >= sizeof(buf) - 1) {
    // Too long to fit or not properly terminated.
    return false;
  }
  memcpy(buf, sentence, len);
  buf[len] = '\0';

  // 2) Tokenize.
  //    Split on commas, preserving empty fields.
  char *tokens[GNGGA_TOKEN_COUNT] = {0};
  int token_count = nmea_split_preserve_empty(buf, tokens, GNGGA_TOKEN_COUNT);

  // 3) Validate tokens.
  //    Expecting tokens to exist (even empty).
  if (token_count < GNGGA_TOKEN_COUNT) {
    return false;
  }
  // Check that mandatory string lengths are non‐zero.
  for (int i = 1; i <= 12; i++) {
    if (tokens[i][0] == '\0') {
      return false;
    }
  }
  char *endptr = NULL;

  // 4) Time "hhmmss.ss".
  //    Convert to hh/mm/ss integers. Ignore fractional seconds.
  float utc_raw = strtof(tokens[1], NULL); // Convert "hhmmss.ss".
  uint8_t hh = (uint8_t)(utc_raw / 10000.0f);
  uint8_t mm = (uint8_t)((utc_raw - ((float)hh * 10000.0f)) / 100.0f);
  uint8_t ss =
      (uint8_t)(utc_raw - ((float)hh * 10000.0f) - ((float)mm * 100.0f));
  gps_data.hour = hh;
  gps_data.minute = mm;
  gps_data.second = ss;

  // 5) Latitude.
  //    tokens[2] = ddmm.mmmmm (string).
  //    tokens[3] = 'N' or 'S'.
  gps_data.latitude = to_decimal_deg(tokens[2], tokens[3][0]);
  gps_data.lat_dir = tokens[3][0];

  // 6) Longitude.
  //    tokens[4] = dddmm.mmmmm (string).
  //    tokens[5] = 'E' or 'W'.
  gps_data.longitude = to_decimal_deg(tokens[4], tokens[5][0]);
  gps_data.lon_dir = tokens[5][0];

  // 7) Fix quality.
  gps_data.position_flags.quality = (unsigned)strtoul(tokens[6], &endptr, 10);
  if (endptr == tokens[6])
    return false;

  // 8) Number of satellites.
  gps_data.satellites = (unsigned)strtoul(tokens[7], &endptr, 10);
  if (endptr == tokens[7])
    return false;

  // 9) Horizontal Dilution of Precision (HDOP).
  gps_data.hdop = strtof(tokens[8], &endptr);

  // 10) Altitude (m).
  gps_data.altitude_m = strtof(tokens[9], &endptr);

  // 11) Geoid separation (m).
  gps_data.geoid_sep_m = strtof(tokens[11], &endptr);

  // 12) Update position fix classification.
  gps_data.position_fix = classify_position_fix(&gps_data.position_flags);

  return true;
}

/**
 * @brief Parse GNRMC fields.
 *
 * @param sentence Pointer to a null-terminated NMEA sentence string.
 *
 * @return == true -> All values seem valid.
 * @return == false -> At least 1 value seems invalid.
 *
 * @note GPS data is still updated on failure using information processed up to
 *       (but not including) the invalid information.
 *
 *  tokens[0] = xxRMC         (string).
 *  tokens[1] = UTC time      (hhmmss.ss).
 *  tokens[2] = status        (character).
 *  tokens[3] = latitude      (ddmm.mmmmm).
 *  tokens[4] = N/S indicator ('N' or 'S').
 *  tokens[5] = longitude     (dddmm.mmmmm).
 *  tokens[6] = E/W indicator ('E' or 'W').
 *  tokens[7] = speed         (float string).
 *  tokens[8] = course        (float string).
 *  tokens[9] = date          (ddmmyy).
 * tokens[10] = magneticVar   (float string).
 * tokens[11] = E/W indicator ('E' or 'W').
 * tokens[12] = position mode (character).
 * tokens[13] = nave status   (character).
 * tokens[14] = checksum      (hexadecimal string with leading '*').
 * tokens[15] = CRLF          (character).
 */
static bool parse_gnrmc(const char *sentence) {
  // 1) Copy into a local buffer for strtok_r.
  char buf[UBLOX_RX_BUFFER_SIZE];
  size_t len = strnlen(sentence, sizeof(buf) - 1);
  if (len >= sizeof(buf) - 1) {
    // Too long to fit or not properly terminated.
    return false;
  }
  memcpy(buf, sentence, len);
  buf[len] = '\0';

  // 2) Tokenize.
  //    Split on commas, preserving empty fields.
  char *tokens[GNRMC_TOKEN_COUNT] = {0};
  int token_count = nmea_split_preserve_empty(buf, tokens, GNRMC_TOKEN_COUNT);

  // 3) Validate tokens.
  //    Expecting tokens to exist (even empty).
  if (token_count < GNRMC_TOKEN_COUNT) {
    return false;
  }
  // Check that mandatory string lengths are non‐zero.
  if (tokens[1][0] == '\0' || tokens[2][0] == '\0' || tokens[3][0] == '\0' ||
      tokens[4][0] == '\0' || tokens[5][0] == '\0' || tokens[6][0] == '\0' ||
      tokens[7][0] == '\0' || tokens[9][0] == '\0' || tokens[12][0] == '\0' ||
      tokens[13][0] == '\0') {
    return false;
  }
  char *endptr = NULL;

  // 4) Time "hhmmss.ss".
  //    Convert to hh/mm/ss integers. Ignore fractional seconds.
  float utc_raw = strtof(tokens[1], NULL); // Convert "hhmmss.ss".
  uint8_t hh = (uint8_t)(utc_raw / 10000.0f);
  uint8_t mm = (uint8_t)((utc_raw - ((float)hh * 10000.0f)) / 100.0f);
  uint8_t ss =
      (uint8_t)(utc_raw - ((float)hh * 10000.0f) - ((float)mm * 100.0f));
  gps_data.hour = hh;
  gps_data.minute = mm;
  gps_data.second = ss;

  // 5) Status.
  char status = tokens[2][0];
  gps_data.position_flags.status = status;
  if (status != 'A' && status != 'V') {
    return false;
  }

  // 6) Latitude.
  //    tokens[3] = ddmm.mmmmm (string).
  //    tokens[4] = 'N' or 'S'.
  gps_data.latitude = to_decimal_deg(tokens[3], tokens[4][0]);
  gps_data.lat_dir = tokens[4][0];

  // 7) Longitude.
  //    tokens[5] = dddmm.mmmmm (string).
  //    tokens[6] = 'E' or 'W'.
  gps_data.longitude = to_decimal_deg(tokens[5], tokens[6][0]);
  gps_data.lon_dir = tokens[6][0];

  // 8) Speed over ground (knots).
  float speed_kn = strtof(tokens[7], &endptr);
  if (endptr == tokens[7]) {
    return false;
  }
  gps_data.speed_knots = speed_kn;

  // 9) Course over ground (degrees).
  float course_deg = strtof(tokens[8], &endptr);
  if (endptr != tokens[8]) {
    gps_data.course_deg = course_deg;
  }

  // 10) Date "ddmmyy".
  int date_raw = (int)strtol(tokens[9], &endptr, 10);
  if (endptr == tokens[9] || date_raw < 0) {
    return false;
  }
  uint8_t day = (uint8_t)(date_raw / 10000);
  uint8_t month = (uint8_t)((date_raw - (day * 10000)) / 100);
  uint8_t year = (uint8_t)(date_raw - (day * 10000) - (month * 100));
  gps_data.day = day;
  gps_data.month = month;
  gps_data.year = year; // 2 digit year ("00" = 2000, "23" = 2023, etc).

  // 11) Position mode indicator (optional-only NMEA 2.3+).
  if (tokens[12]) {
    gps_data.position_flags.pos_mode = tokens[12][0];
  }

  // 12) Navigation status (optional-only NMEA 4.10+).
  // TODO: Skipped implementation.
  //  if (tokens[13]) {
  //    gps_data.nav_status = tokens[13][0];
  //  }

  // 13) Update position fix classification.
  gps_data.position_fix = classify_position_fix(&gps_data.position_flags);

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
  } else if (strncmp(sentence, "$GNRMC", 6) == 0) { // Handle GNRMC sentence.
    if (!validate_nmea_checksum(sentence) || !parse_gnrmc(sentence)) {
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
    // Check how many bytes have been written by DMA since last time.
    uint16_t pos = UBLOX_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    // Process every new byte in order.
    while (ublox_rx_index != pos) {
      uint8_t b = ublox_rx_dma_buffer[ublox_rx_index];
      ublox_process_byte(b, ublox_rx_index);
      ublox_rx_index = (ublox_rx_index + 1) % UBLOX_RX_BUFFER_SIZE;
    }

    __HAL_UART_CLEAR_IDLEFLAG(huart); // Clear the IDLE flag.

    // Rearm DMA receive.
    HAL_UART_Receive_DMA(&UBLOX_HUART, ublox_rx_dma_buffer,
                         UBLOX_RX_BUFFER_SIZE);
  }
}

/** Public functions. *********************************************************/

void ublox_init(void) {
  // Ensure the u-blox module is not in reset state.
  HAL_GPIO_WritePin(UBLOX_RESETN_PORT, UBLOX_RESETN_PIN, GPIO_PIN_SET);

  // Ensure u-blox module is at same starting baud rate as the STM32.
  ublox_set_baud_rate(9600);
  HAL_Delay(5);

  // Swap to a higher baud rate on the u-blox module, then on the STM32.
  ublox_set_baud_rate(UBLOX_INIT_BAUD_RATE);
  UBLOX_HUART.Init.BaudRate = UBLOX_INIT_BAUD_RATE;
  HAL_UART_Init(&UBLOX_HUART);
  HAL_Delay(5);

  // Set dynamic model.
  ublox_set_dynamic_model(0);
  HAL_Delay(5);

  // Disable unused NMEA messages.
  ublox_disable_other_nmea_messages();
  HAL_Delay(5);

  // Enable 10 Hz measurement and 10 Hz (GGA and RMC) messages.
  ublox_enable_10hz();
  HAL_Delay(5);

  // Start UART reception with DMA and enable IDLE based interrupts.
  HAL_UART_Receive_DMA(&UBLOX_HUART, ublox_rx_dma_buffer, UBLOX_RX_BUFFER_SIZE);
  __HAL_UART_ENABLE_IT(&UBLOX_HUART, UART_IT_IDLE);
}

void ublox_reset(void) {
  HAL_GPIO_WritePin(UBLOX_RESETN_PORT, UBLOX_RESETN_PIN, GPIO_PIN_RESET);
  HAL_Delay(5); // Hold reset for 5 ms.
  HAL_GPIO_WritePin(UBLOX_RESETN_PORT, UBLOX_RESETN_PIN, GPIO_PIN_SET);
}

void ublox_set_baud_rate(uint32_t baud_rate) {
  // 1) Build the core body of the PUBX,41 sentence (no '$' and no "*CS"),
  //    e.g. "PUBX,41,1,0007,0003,9600,0".
  char core_msg[64];
  int core_len =
      snprintf(core_msg, sizeof(core_msg), "PUBX,41,1,0007,0003,%lu,0",
               (unsigned long)baud_rate);
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

  // 4) Send it out via HAL_UART (blocking).
  HAL_UART_Transmit(&UBLOX_HUART, (uint8_t *)full_sentence, (uint16_t)full_len,
                    HAL_MAX_DELAY);

  // NOTE: After sending this string, the u-blox UART hardware switches
  //       to `new_baud`. If you want to continue communicating at new_baud,
  //       you must reconfigure huartx.Init.BaudRate = new_baud and call
  //       HAL_UART_Init(&huartx) on the STM32 side.
}

void ublox_set_dynamic_model(uint8_t dyn_model) {
  // 1) Construct the 36-byte payload for UBX-CFG-NAV5:
  //    Offset 0-1 : mask (0x0001 -> little endian: 0x01, 0x00).
  //    Offset 2   : dynModel.
  //    Offset 3   : fixMode = 0x03 (Auto 2D/3D).
  //    Offset 4-35: all zeros.
  uint8_t payload[36];
  memset(payload, 0x00, sizeof(payload));
  payload[0] = 0x01;      // mask LSB.
  payload[1] = 0x00;      // mask MSB  (-> 0x0001).
  payload[2] = dyn_model; // dynModel = user choice.
  payload[3] = 0x03;      // fixMode = 3 (Auto 2D/3D).

  // 2) Build the UBX header + length (class=0x06, id=0x24, length=36).
  //    Sync chars: 0xB5, 0x62.
  //    Class      : 0x06.
  //    ID         : 0x24.
  //    Length LSB : 0x24 (36 decimal).
  //    Length MSB : 0x00.
  uint8_t header[6] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00};

  // 3) Allocate a single TX buffer:
  uint8_t tx_buf[44]; // header(6) + payload(36) + CK_A/CK_B(2) = 44 bytes.
  memcpy(&tx_buf[0], header, sizeof(header));   // Copy header.
  memcpy(&tx_buf[6], payload, sizeof(payload)); // Copy payload.

  // 4) Compute checksum:
  // Range is (class, id, length, payload) = txBuf[2..(2+4+36-1)] = 40 bytes.
  // Start at txBuf[2], length = 4 (class+id+len) + 36 (payload) = 40.
  uint8_t ck_a, ck_b;
  compute_ubx_checksum(&tx_buf[2], 40, &ck_a, &ck_b);

  // 5) Append CK_A, CK_B.
  tx_buf[42] = ck_a;
  tx_buf[43] = ck_b;

  // 6) Send it out via HAL_UART (blocking).
  HAL_UART_Transmit(&UBLOX_HUART, tx_buf, (uint16_t)sizeof(tx_buf),
                    HAL_MAX_DELAY);
}

void ublox_disable_other_nmea_messages(void) {
  // 1) List of NMEA msgIDs to disable (all except 0x00=GGA and 0x04=RMC).
  const uint8_t disable_list[] = {
      0x01, // GLL.
      0x02, // GSA.
      0x03, // GSV.
      0x05, // VTG.
      0x07, // GST.
      0x08, // ZDA.
      0x09, // GBS (GNSS Satellite Fault Detection).
      0x0A, // DTM (Datum reference).
      0x0D, // GRS (GNSS Range Residuals).
            // (skip 0x00=GGA, 0x04=RMC)
  };
  const size_t n = sizeof(disable_list) / sizeof(disable_list[0]);

  // 2) For each message ID, build a 14-byte CFG-MSG UBX packet and transmit it.
  for (size_t i = 0; i < n; i++) {
    uint8_t msgID = disable_list[i];

    // a) Prepare header + payload (14 bytes total including CKs at [12],[13]).
    uint8_t packet[14] = {
        0xB5,
        0x62, // UBX sync chars.
        0x06,
        0x01, // class = CFG (0x06), id = MSG (0x01).
        0x08,
        0x00, // payload length = 8 (LSB=0x08, MSB=0x00).
        0xF0,
        msgID, // payload[0] = msgClass=NMEA (0xF0).
               // payload[1] = msgID (disable_list[i]).
        0x00,  // payload[2] = rateUART = 0 (disable).
        0x00,  // payload[3] = rateI2C = 0.
        0x00,  // payload[4] = rateUSB = 0.
        0x00,  // payload[5] = rateSPI = 0.
        0x00,
        0x00 // payload[6..7] = reserved = 0x0000.
             // packet[14], packet[15] = checksum (to be computed).
    };

    // b) Compute checksum over bytes [2..11] (class, id, length, payload[0..5],
    //    reserved bytes).
    //
    //    Actually: (2..(2+4+8-1)) = indices [2..13], but since our packet[] is
    //              14 bytes (0..13), compute over packet[2]..packet[13], then
    //              store results at packet[12],packet[13].
    //
    //    But note: 6(header) + 8(payload) = 14. Check sum begins at index=2 for
    //              4 + 8 = 12 bytes:
    //      [2] class
    //      [3] id
    //      [4] length LSB
    //      [5] length MSB
    //      [6..13] entire 8-byte payload
    //
    //    Thus: check sum over 12 bytes: packet[2]..packet[13], and store CK at
    //          [14],[15], but our array is only 14 long. To fix indexing:
    //          packet[] is 14 bytes (0..13), so the payload ends at [11].
    //          0x00,0x00 is stored at [12],[13] as "reserved" - but must append
    //          CK after that, so a 16-byte array is needed.
    uint8_t tx_buf[16];         // Switch to a 16-byte buffer for simplicity.
    memcpy(tx_buf, packet, 12); // Copy first 12 bytes (0..11).
    tx_buf[12] = packet[12];    // Reserved LSB.
    tx_buf[13] = packet[13];    // Reserved MSB.
    // Compute checksum over tx_buf[2..13].
    uint8_t ck_a = 0, ck_b = 0;
    compute_ubx_checksum(&tx_buf[2], 12, &ck_a, &ck_b);

    // c) Append CK_A, CK_B at indices [14], [15].
    tx_buf[14] = ck_a;
    tx_buf[15] = ck_b;

    // d) Transmit all 16 bytes.
    HAL_UART_Transmit(&UBLOX_HUART, tx_buf, 16, HAL_MAX_DELAY);
  }
}

void ublox_enable_10hz(void) {
  uint8_t ck_a, ck_b;
  uint8_t packet[16];

  // 1) CFG-RATE: set measRate=100ms (0x0064), navRate=1, timeRef=0 (UTC).
  uint8_t rate_payload[10] = {0x06, 0x08, 0x06, 0x00, 0x64,
                              0x00, 0x01, 0x00, 0x00, 0x00};
  // Compute checksum over the 10 bytes.
  compute_ubx_checksum(rate_payload, sizeof(rate_payload), &ck_a, &ck_b);
  // Build full 12-byte packet (header + payload + checksum).
  packet[0] = 0xB5;
  packet[1] = 0x62;
  memcpy(&packet[2], rate_payload, sizeof(rate_payload));
  packet[12] = ck_a;
  packet[13] = ck_b;
  // Transmit 14 bytes.
  HAL_UART_Transmit(&UBLOX_HUART, packet, 14, HAL_MAX_DELAY);
  HAL_Delay(5);

  // 3) Enable GGA @ 10 Hz.
  uint8_t gga_payload[12] = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x00,
                             0x0A, 0x00, 0x00, 0x00, 0x00, 0x00};
  // Compute checksum over class/id/len + payload (10 bytes).
  compute_ubx_checksum(&gga_payload[2], 10, &ck_a, &ck_b);
  packet[0] = 0xB5;
  packet[1] = 0x62;
  memcpy(&packet[2], gga_payload, 12);
  packet[14] = ck_a;
  packet[15] = ck_b;
  // Transmit 16 bytes.
  HAL_UART_Transmit(&UBLOX_HUART, packet, 16, HAL_MAX_DELAY);
  HAL_Delay(5);

  // 4) Enable RMC @ 10 Hz.
  uint8_t rmc_payload[12] = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x04,
                             0x0A, 0x00, 0x00, 0x00, 0x00, 0x00};
  // Compute checksum over class/id/len + payload (10 bytes).
  compute_ubx_checksum(&rmc_payload[2], 10, &ck_a, &ck_b);
  packet[0] = 0xB5;
  packet[1] = 0x62;
  memcpy(&packet[2], rmc_payload, 12);
  packet[14] = ck_a;
  packet[15] = ck_b;
  // Transmit 16 bytes.
  HAL_UART_Transmit(&UBLOX_HUART, packet, 16, HAL_MAX_DELAY);
  HAL_Delay(5);
}
