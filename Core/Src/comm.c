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
#include "bmp390_runner.h"
#include "bno085_runner.h"
#include "configuration.h"
#include "ublox_hal_uart.h"
#include "ws2812b_hal_pwm.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/** Definitions. **************************************************************/

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
    printf("%s %u.%u.%u.%c\r\n", SCALPELSPACE_SHORT_NAME,
           MOMENTUM_VERSION_MAJOR, MOMENTUM_VERSION_MINOR,
           MOMENTUM_VERSION_PATCH, MOMENTUM_VERSION_IDENTIFIER);
  }

  else if (strncmp(line, "rgb", 3) == 0) {
    unsigned int r_raw, g_raw, b_raw;
    // Accept formats like:
    // "rgb 255,0,128"
    // "rgb 255, 0, 128"
    if (sscanf(line + 3, "%u , %u , %u", &r_raw, &g_raw, &b_raw) == 3) {
      if (r_raw <= 255 && g_raw <= 255 && b_raw <= 255) {
        uint8_t r = (uint8_t)r_raw;
        uint8_t g = (uint8_t)g_raw;
        uint8_t b = (uint8_t)b_raw;
        ws2812b_set_colour(0, r, g, b);
        ws2812b_update();
      } else {
        printf("Error: RGB values must be [0,255]\r\n");
      }
    } else {
      printf("Error: expected `rgb <R>, <G>, <B>`\r\n");
    }
  }

  else if (strcmp(line, "imu") == 0) {
    printf("%.3f i,%.3f j,%.3f k, %.3f r\r\n", bno085_quaternion_i,
           bno085_quaternion_j, bno085_quaternion_k, bno085_quaternion_real);
  }

  else if (strcmp(line, "baro") == 0) {
    printf("%.3f degC,%.3f Pa\r\n", bmp390_temperature, bmp390_pressure);
  }

  else if (strcmp(line, "gnss") == 0) {
    printf("%u/%u/%u %u:%u:%u\r\n", gps_data.year + 2000, gps_data.month,
           gps_data.day, gps_data.hour, gps_data.minute, gps_data.second);
    printf("%.3f (%c),%.3f (%c), %.3f m\r\n", gps_data.latitude,
           gps_data.lat_dir, gps_data.longitude, gps_data.lon_dir,
           gps_data.altitude_m);
  }

  else if (strcmp(line, "report") == 0) {
    printf("%.3f i,%.3f j,%.3f k, %.3f r\r\n", bno085_quaternion_i,
           bno085_quaternion_j, bno085_quaternion_k, bno085_quaternion_real);
    printf("%.3f degC,%.3f Pa\r\n", bmp390_temperature, bmp390_pressure);
    printf("%u/%u/%u %u:%u:%u\r\n", gps_data.year + 2000, gps_data.month,
           gps_data.day, gps_data.hour, gps_data.minute, gps_data.second);
    printf("%.3f (%c),%.3f (%c), %.3f m\r\n", gps_data.latitude,
           gps_data.lat_dir, gps_data.longitude, gps_data.lon_dir,
           gps_data.altitude_m);
  }

  else { // Error: unknown command.
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
