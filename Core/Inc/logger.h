/*******************************************************************************
 * @file logger.h
 * @brief Momentum sensor data logging to NVM memory.
 *******************************************************************************/

#ifndef MOMENTUM__LOGGER_H
#define MOMENTUM__LOGGER_H

/** Includes. *****************************************************************/

#include "stm32l4xx_hal.h"

/** Definitions. **************************************************************/

#define INVALID_ADDRESS 0xFFFFFFFFU

// Valid log page mark.
#define LOG_MAGIC 0xA5A5A5A5U

/** Definitions. **************************************************************/

// Header stored at the start of each 256-byte page.
typedef struct {
  uint32_t magic;      // LOG_MAGIC.
  uint32_t session_id; // User-supplied per-execution ID.
  uint32_t page_seq;   // Increments at each new page within the NVM.
} log_page_header_t;

/** Public functions. *********************************************************/

/**
 * @brief Initialize the logger within an address range and tag via session ID.
 *
 * @param start_address Start of NVM address range.
 * @param end_address End of NVM address range.
 * @param session_id Session ID to mark data logging.
 */
void logger_init(uint32_t start_address, uint32_t end_address,
                 uint32_t session_id);

/**
 * @brief Erase the entire region, resetting everything.
 */
void logger_hard_reset(void);

/**
 * @brief Append one buffer of length len into flash.
 *
 * Automatically spans pages, skips already-used pages, and never overwriting
 * data.
 *
 * @param buf Pointer to the data_array.
 * @param len Length of data in bytes to program.
 */
void logger_write(const uint8_t *buf, uint16_t len);

void log_quaternion(void);
void log_gyro(void);
void log_accel(void);
void log_lin_accel(void);
void log_gravity(void);
void log_pressure_temp(void);
void log_gps_datetime(void);
void log_gps_coord(void);
void log_gps_altitude_speed(void);
void log_gps_heading(void);
void log_gps_stats(void);

#endif
