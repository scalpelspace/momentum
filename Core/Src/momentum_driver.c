/*******************************************************************************
 * @file momentum_driver.c
 * @brief Momentum driver for SPI based communication.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "momentum_driver.h"

/** Definitions. **************************************************************/

/** Public functions. *********************************************************/

void pack_sensor_data_8bit(uint8_t *data_array, sensor_data_t data) {
  size_t offset = 0;

#define PACK_FLOAT(f)                                                          \
  do {                                                                         \
    memcpy(&data_array[offset], &(f), sizeof(float));                          \
    offset += sizeof(float);                                                   \
  } while (0)

#define PACK_DOUBLE(d)                                                         \
  do {                                                                         \
    memcpy(&data_array[offset], &(d), sizeof(double));                         \
    offset += sizeof(double);                                                  \
  } while (0)

#define PACK_BYTE(b) data_array[offset++] = (uint8_t)(b)

  // Pack BNO085.
  PACK_FLOAT(data.bno085_quaternion_i);
  PACK_FLOAT(data.bno085_quaternion_j);
  PACK_FLOAT(data.bno085_quaternion_k);
  PACK_FLOAT(data.bno085_quaternion_real);
  PACK_FLOAT(data.bno085_quaternion_accuracy_rad);
  PACK_FLOAT(data.bno085_quaternion_accuracy_deg);
  PACK_FLOAT(data.bno085_gyro_x);
  PACK_FLOAT(data.bno085_gyro_y);
  PACK_FLOAT(data.bno085_gyro_z);
  PACK_FLOAT(data.bno085_accel_x);
  PACK_FLOAT(data.bno085_accel_y);
  PACK_FLOAT(data.bno085_accel_z);
  PACK_FLOAT(data.bno085_lin_accel_x);
  PACK_FLOAT(data.bno085_lin_accel_y);
  PACK_FLOAT(data.bno085_lin_accel_z);
  PACK_FLOAT(data.bno085_gravity_x);
  PACK_FLOAT(data.bno085_gravity_y);
  PACK_FLOAT(data.bno085_gravity_z);

  // Pack BMP390.
  PACK_DOUBLE(data.bmp390_temperature);
  PACK_DOUBLE(data.bmp390_pressure);

  // Pack GPS.
  PACK_BYTE(data.gps_hour);
  PACK_BYTE(data.gps_minute);
  PACK_BYTE(data.gps_second);
  PACK_BYTE(data.gps_day);
  PACK_BYTE(data.gps_month);
  PACK_BYTE(data.gps_year);
  PACK_DOUBLE(data.gps_latitude);
  PACK_BYTE(data.gps_lat_dir);
  PACK_DOUBLE(data.gps_longitude);
  PACK_BYTE(data.gps_lon_dir);
  PACK_BYTE(data.gps_fix_quality);
  PACK_BYTE(data.gps_satellites);
  PACK_FLOAT(data.gps_hdop);
  PACK_DOUBLE(data.gps_altitude);
  PACK_DOUBLE(data.gps_geoid_sep);

#undef PACK_FLOAT
#undef PACK_DOUBLE
#undef PACK_ARRAY
#undef PACK_BYTE
}

void unpack_sensor_data_8bit(const uint8_t *data_array, sensor_data_t *data) {
  size_t offset = 0;

#define UNPACK_FLOAT(f)                                                        \
  do {                                                                         \
    memcpy(&(f), &data_array[offset], sizeof(float));                          \
    offset += sizeof(float);                                                   \
  } while (0)

#define UNPACK_DOUBLE(d)                                                       \
  do {                                                                         \
    memcpy(&(d), &data_array[offset], sizeof(double));                         \
    offset += sizeof(double);                                                  \
  } while (0)

#define UNPACK_BYTE(b) (b) = data_array[offset++]

  // Unpack BNO085.
  UNPACK_FLOAT(data->bno085_quaternion_i);
  UNPACK_FLOAT(data->bno085_quaternion_j);
  UNPACK_FLOAT(data->bno085_quaternion_k);
  UNPACK_FLOAT(data->bno085_quaternion_real);
  UNPACK_FLOAT(data->bno085_quaternion_accuracy_rad);
  UNPACK_FLOAT(data->bno085_quaternion_accuracy_deg);
  UNPACK_FLOAT(data->bno085_gyro_x);
  UNPACK_FLOAT(data->bno085_gyro_y);
  UNPACK_FLOAT(data->bno085_gyro_z);
  UNPACK_FLOAT(data->bno085_accel_x);
  UNPACK_FLOAT(data->bno085_accel_y);
  UNPACK_FLOAT(data->bno085_accel_z);
  UNPACK_FLOAT(data->bno085_lin_accel_x);
  UNPACK_FLOAT(data->bno085_lin_accel_y);
  UNPACK_FLOAT(data->bno085_lin_accel_z);
  UNPACK_FLOAT(data->bno085_gravity_x);
  UNPACK_FLOAT(data->bno085_gravity_y);
  UNPACK_FLOAT(data->bno085_gravity_z);

  // Unpack BMP390.
  UNPACK_DOUBLE(data->bmp390_temperature);
  UNPACK_DOUBLE(data->bmp390_pressure);

  // Unpack GPS.
  UNPACK_BYTE(data->gps_hour);
  UNPACK_BYTE(data->gps_minute);
  UNPACK_BYTE(data->gps_second);
  UNPACK_BYTE(data->gps_day);
  UNPACK_BYTE(data->gps_month);
  UNPACK_BYTE(data->gps_year);
  UNPACK_DOUBLE(data->gps_latitude);
  UNPACK_BYTE(data->gps_lat_dir);
  UNPACK_DOUBLE(data->gps_longitude);
  UNPACK_BYTE(data->gps_lon_dir);
  UNPACK_BYTE(data->gps_fix_quality);
  UNPACK_BYTE(data->gps_satellites);
  UNPACK_FLOAT(data->gps_hdop);
  UNPACK_DOUBLE(data->gps_altitude);
  UNPACK_DOUBLE(data->gps_geoid_sep);

#undef UNPACK_FLOAT
#undef UNPACK_DOUBLE
#undef UNPACK_BYTE
}
