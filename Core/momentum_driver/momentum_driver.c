/*******************************************************************************
 * @file momentum_driver.c
 * @brief Momentum driver for SPI based communication.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "momentum_driver.h"

/** Definitions. **************************************************************/

#define MOMENTUM_START_OF_FRAME 0xAA

#define MOMENTUM_FRAME_TYPE_IMU_QUAT 0x11
#define MOMENTUM_FRAME_TYPE_IMU_GYRO 0x12
#define MOMENTUM_FRAME_TYPE_IMU_ACCEL 0x13
#define MOMENTUM_FRAME_TYPE_IMU_LINACCEL 0x14
#define MOMENTUM_FRAME_TYPE_IMU_GRAV 0x15
#define MOMENTUM_FRAME_TYPE_BAR_ENV 0x16
#define MOMENTUM_FRAME_TYPE_GPS_COORD 0x17
#define MOMENTUM_FRAME_TYPE_GPS_DATETIME 0x18
#define MOMENTUM_FRAME_TYPE_GPS_STATUS 0x19

/** Private functions. ********************************************************/

static inline uint8_t *pack_uint_8(uint8_t *p, uint8_t v) {
  *p++ = v;
  return p + 1;
}

static inline uint8_t *pack_char_8(uint8_t *p, char c) {
  *p = (uint8_t)c;
  return p + 1;
}

static inline uint8_t *pack_float_32(uint8_t *p, float v) {
  memcpy(p, &v, 4);
  return p + 4;
}

static inline uint8_t *pack_double_64(uint8_t *p, double v) {
  memcpy(p, &v, 8);
  return p + 8;
}

/** Public functions. *********************************************************/

uint8_t build_quaternion_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *p = f->payload;
  p = pack_float_32(p, s->bno085_quaternion_i);
  p = pack_float_32(p, s->bno085_quaternion_j);
  p = pack_float_32(p, s->bno085_quaternion_k);
  p = pack_float_32(p, s->bno085_quaternion_real);
  p = pack_float_32(p, s->bno085_quaternion_accuracy_rad);
  p = pack_float_32(p, s->bno085_quaternion_accuracy_deg);
  return (uint8_t)(p - f->payload);
}

uint8_t build_gyro_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *p = f->payload;
  p = pack_float_32(p, s->bno085_gyro_x);
  p = pack_float_32(p, s->bno085_gyro_y);
  p = pack_float_32(p, s->bno085_gyro_z);
  return (uint8_t)(p - f->payload);
}

uint8_t build_accel_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *p = f->payload;
  p = pack_float_32(p, s->bno085_accel_x);
  p = pack_float_32(p, s->bno085_accel_y);
  p = pack_float_32(p, s->bno085_accel_z);
  return (uint8_t)(p - f->payload);
}

uint8_t build_lin_accel_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *p = f->payload;
  p = pack_float_32(p, s->bno085_lin_accel_x);
  p = pack_float_32(p, s->bno085_lin_accel_y);
  p = pack_float_32(p, s->bno085_lin_accel_z);
  return (uint8_t)(p - f->payload);
}

uint8_t build_gravity_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *p = f->payload;
  p = pack_float_32(p, s->bno085_gravity_x);
  p = pack_float_32(p, s->bno085_gravity_y);
  p = pack_float_32(p, s->bno085_gravity_z);
  return (uint8_t)(p - f->payload);
}

uint8_t build_pressure_temp_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *p = f->payload;
  p = pack_double_64(p, s->bmp390_temperature);
  p = pack_double_64(p, s->bmp390_pressure);
  return (uint8_t)(p - f->payload);
}

uint8_t build_gps_datetime_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *p = f->payload;
  p = pack_uint_8(p, s->gps_hour);
  p = pack_uint_8(p, s->gps_minute);
  p = pack_uint_8(p, s->gps_second);
  p = pack_uint_8(p, s->gps_day);
  p = pack_uint_8(p, s->gps_month);
  p = pack_uint_8(p, s->gps_year);
  return (uint8_t)(p - f->payload);
}

uint8_t build_gps_coord_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *p = f->payload;
  p = pack_double_64(p, s->gps_latitude);
  p = pack_char_8(p, s->gps_lat_dir);
  p = pack_double_64(p, s->gps_longitude);
  p = pack_char_8(p, s->gps_lon_dir);
  return (uint8_t)(p - f->payload);
}

uint8_t build_gps_stats_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *p = f->payload;
  p = pack_uint_8(p, s->gps_fix_quality);
  p = pack_uint_8(p, s->gps_satellites);
  p = pack_float_32(p, s->gps_hdop);
  p = pack_double_64(p, s->gps_altitude);
  p = pack_double_64(p, s->gps_geoid_sep);
  return (uint8_t)(p - f->payload);
}
