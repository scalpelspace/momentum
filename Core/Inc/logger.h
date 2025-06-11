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
