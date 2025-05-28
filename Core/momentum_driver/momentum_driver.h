/*******************************************************************************
 * @file momentum_driver.h
 * @brief Momentum driver for SPI based communication.
 *******************************************************************************
 */

#ifndef MOMENTUM__DRIVER_H
#define MOMENTUM__DRIVER_H

/** Includes. *****************************************************************/

#include <stdint.h>
#include <string.h>

/** Definitions. **************************************************************/

#define MOMENTUM_MAX_DATA_SIZE 32

/** Public types. *************************************************************/

/**
 * @brief Momentum sensor hub SPI communication frame.
 */
typedef struct __attribute__((packed)) {
  uint8_t start_of_frame;                  // Start of frame (SOF) for syncing.
  uint8_t frame_type;                      // Frame type identifier.
  uint8_t sequence;                        // Roll-over counter.
  uint8_t length;                          // Number of data payload bytes.
  uint8_t payload[MOMENTUM_MAX_DATA_SIZE]; // Data payload.
  uint16_t crc;                            // CRC-16 of frame_type...data[-1].
} momentum_frame_t;

typedef struct {
  float bno085_quaternion_i;
  float bno085_quaternion_j;
  float bno085_quaternion_k;
  float bno085_quaternion_real;
  float bno085_quaternion_accuracy_rad;
  float bno085_quaternion_accuracy_deg;
  float bno085_gyro_x;
  float bno085_gyro_y;
  float bno085_gyro_z;
  float bno085_accel_x;
  float bno085_accel_y;
  float bno085_accel_z;
  float bno085_lin_accel_x;
  float bno085_lin_accel_y;
  float bno085_lin_accel_z;
  float bno085_gravity_x;
  float bno085_gravity_y;
  float bno085_gravity_z;
  double bmp390_temperature;
  double bmp390_pressure;
  uint8_t gps_hour;        // 0-23.
  uint8_t gps_minute;      // 0-59.
  uint8_t gps_second;      // 0-59.
  uint8_t gps_day;         // 1-31.
  uint8_t gps_month;       // 1-12.
  uint8_t gps_year;        // Year since 2000 (25 for 2025).
  double gps_latitude;     // Latitude in decimal degrees.
  char gps_lat_dir;        // Latitude Direction (N/S).
  double gps_longitude;    // Longitude in decimal degrees.
  char gps_lon_dir;        // Longitude Direction (E/W).
  uint8_t gps_fix_quality; // GPS Fix Quality.
  //  0 = No fix.
  //  1 = Autonomous GNSS fix.
  //  2 = Differential GNSS fix.
  //  4 = RTK fixed.
  //  5 = RTK float.
  //  6 = Estimated/dead reckoning fix.
  uint8_t gps_satellites; // Number of Satellites.
  float gps_hdop;         // Horizontal Dilution of Precision.
  double gps_altitude;    // Altitude in meters.
  double gps_geoid_sep;   // Geoidal Separation.
} sensor_data_t;

/** Public functions. *********************************************************/

uint8_t build_quaternion_payload(momentum_frame_t *f, sensor_data_t *s);
uint8_t build_gyro_payload(momentum_frame_t *f, sensor_data_t *s);
uint8_t build_accel_payload(momentum_frame_t *f, sensor_data_t *s);
uint8_t build_lin_accel_payload(momentum_frame_t *f, sensor_data_t *s);
uint8_t build_gravity_payload(momentum_frame_t *f, sensor_data_t *s);
uint8_t build_pressure_temp_payload(momentum_frame_t *f, sensor_data_t *s);
uint8_t build_gps_datetime_payload(momentum_frame_t *f, sensor_data_t *s);
uint8_t build_gps_coord_payload(momentum_frame_t *f, sensor_data_t *s);
uint8_t build_gps_stats_payload(momentum_frame_t *f, sensor_data_t *s);

#endif
