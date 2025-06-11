/*******************************************************************************
 * @file logger.h
 * @brief Momentum sensor data logging to NVM memory.
 *******************************************************************************/

#ifndef MOMENTUM__LOGGER_H
#define MOMENTUM__LOGGER_H

/** Includes. *****************************************************************/

#include "stm32l4xx_hal.h"

/** Public functions. *********************************************************/

/**
 * @brief Initialize the logger (call once at startup).
 *        Does *not* erase all flash—first write in each sector will erase it.
 */
void logger_init(void);

/**
 * @brief Wipes the entire logging region and resets pointer.
 */
void logger_reset(void);

void log_quaternion_payload(void);
void log_gyro_payload(void);
void log_accel_payload(void);
void log_lin_accel_payload(void);
void log_gravity_payload(void);
void log_pressure_temp_payload(void);
void log_gps_datetime_payload(void);
void log_gps_coord_payload(void);
void log_gps_altitude_speed_payload(void);
void log_gps_heading_payload(void);
void log_gps_stats_payload(void);

#endif
