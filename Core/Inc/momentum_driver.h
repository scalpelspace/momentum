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

// Momentum SPI transaction frame sizes (in bytes):
#define MOMENTUM_MAX_FRAME_SIZE 128 // Total = 128.
#define MOMENTUM_MAX_DATA_SIZE 125  // Data = 125.
// Frame type = 1.
// Length value = 1.
// Checksum = 1.

/** Public types. *************************************************************/

typedef struct {
  uint8_t frame_type;
  uint8_t length;
  uint8_t data[MOMENTUM_MAX_DATA_SIZE];
  uint8_t checksum;
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

/**
 * @brief Pack sensor data into a uint8_t array.
 *
 * @param data_array Pointer to the output array to store packed bytes.
 * @param data The sensor data to serialize.
 */
void pack_sensor_data_8bit(uint8_t *data_array, sensor_data_t data);

/**
 * @brief Unpack sensor data from a uint8_t array.
 *
 * @param data_array Pointer to the input array containing packed data.
 * @param data Pointer to the sensor_data struct to populate.
 */
void unpack_sensor_data_8bit(const uint8_t *data_array, sensor_data_t *data);

#endif
