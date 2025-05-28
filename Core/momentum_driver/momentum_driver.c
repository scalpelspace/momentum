/*******************************************************************************
 * @file momentum_driver.c
 * @brief Momentum driver for SPI based communication.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "momentum_driver.h"

/** Private functions. ********************************************************/

static inline uint8_t *pack_uint_8(uint8_t *p, uint8_t v) {
  *p++ = v;
  return p;
}

static inline uint8_t *pack_char_8(uint8_t *p, char c) {
  *p++ = (uint8_t)c;
  return p;
}

static inline uint8_t *pack_float_32(uint8_t *p, float v) {
  memcpy(p, &v, 4);
  return p + 4;
}

static inline uint8_t *pack_double_64(uint8_t *p, double v) {
  memcpy(p, &v, 8);
  return p + 8;
}

static inline const uint8_t *unpack_uint_8(const uint8_t *p, uint8_t *v) {
  *v = *p;
  return p + 1;
}

static inline const uint8_t *unpack_char_8(const uint8_t *p, char *c) {
  *c = (char)*p;
  return p + 1;
}

static inline const uint8_t *unpack_float_32(const uint8_t *p, float *f) {
  memcpy(f, p, 4);
  return p + 4;
}

static inline const uint8_t *unpack_double_64(const uint8_t *p, double *d) {
  memcpy(d, p, 8);
  return p + 8;
}

static inline uint8_t update_payload_length(momentum_frame_t *f,
                                            const uint8_t *start,
                                            const uint8_t *end) {
  uint8_t len = (uint8_t)(end - start);
  f->length = len;
  return len;
}

/** Public functions. *********************************************************/

uint16_t crc16_ccitt(uint16_t crc, const uint8_t *buf, size_t len) {
  while (len--) {
    crc ^= (uint16_t)(*buf++) << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void build_crc(momentum_frame_t *f) {
  // Check: frame_type(1) + sequence(1) + length(1) + data[length].
  size_t n = 3 + f->length;
  // `&f->frame_type` is the address of first byte to CRC.
  f->crc = crc16_ccitt(MOMENTUM_CRC_INITIAL, &f->frame_type, n);
}

bool verify_crc(const momentum_frame_t *f) {
  // Check: frame_type(1) + sequence(1) + length(1) + data[length].
  size_t n = 3 + f->length;
  uint16_t crc = crc16_ccitt(MOMENTUM_CRC_INITIAL, &f->frame_type, n);
  return (crc == f->crc);
}

uint8_t build_quaternion_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_float_32(p, s->bno085_quaternion_i);
  p = pack_float_32(p, s->bno085_quaternion_j);
  p = pack_float_32(p, s->bno085_quaternion_k);
  p = pack_float_32(p, s->bno085_quaternion_real);
  p = pack_float_32(p, s->bno085_quaternion_accuracy_rad);
  p = pack_float_32(p, s->bno085_quaternion_accuracy_deg);
  return update_payload_length(f, start, p);
}

uint8_t parse_quaternion_payload(const momentum_frame_t *f, sensor_data_t *s) {
  const uint8_t *p = f->payload;
  p = unpack_float_32(p, &s->bno085_quaternion_i);
  p = unpack_float_32(p, &s->bno085_quaternion_j);
  p = unpack_float_32(p, &s->bno085_quaternion_k);
  p = unpack_float_32(p, &s->bno085_quaternion_real);
  p = unpack_float_32(p, &s->bno085_quaternion_accuracy_rad);
  p = unpack_float_32(p, &s->bno085_quaternion_accuracy_deg);
  return (uint8_t)(p - f->payload);
}

uint8_t build_gyro_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_float_32(p, s->bno085_gyro_x);
  p = pack_float_32(p, s->bno085_gyro_y);
  p = pack_float_32(p, s->bno085_gyro_z);
  return update_payload_length(f, start, p);
}

uint8_t parse_gyro_payload(const momentum_frame_t *f, sensor_data_t *s) {
  const uint8_t *p = f->payload;
  p = unpack_float_32(p, &s->bno085_gyro_x);
  p = unpack_float_32(p, &s->bno085_gyro_y);
  p = unpack_float_32(p, &s->bno085_gyro_z);
  return (uint8_t)(p - f->payload);
}

uint8_t build_accel_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_float_32(p, s->bno085_accel_x);
  p = pack_float_32(p, s->bno085_accel_y);
  p = pack_float_32(p, s->bno085_accel_z);
  return update_payload_length(f, start, p);
}

uint8_t parse_accel_payload(const momentum_frame_t *f, sensor_data_t *s) {
  const uint8_t *p = f->payload;
  p = unpack_float_32(p, &s->bno085_accel_x);
  p = unpack_float_32(p, &s->bno085_accel_y);
  p = unpack_float_32(p, &s->bno085_accel_z);
  return (uint8_t)(p - f->payload);
}

uint8_t build_lin_accel_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_float_32(p, s->bno085_lin_accel_x);
  p = pack_float_32(p, s->bno085_lin_accel_y);
  p = pack_float_32(p, s->bno085_lin_accel_z);
  return update_payload_length(f, start, p);
}

uint8_t parse_lin_accel_payload(const momentum_frame_t *f, sensor_data_t *s) {
  const uint8_t *p = f->payload;
  p = unpack_float_32(p, &s->bno085_lin_accel_x);
  p = unpack_float_32(p, &s->bno085_lin_accel_y);
  p = unpack_float_32(p, &s->bno085_lin_accel_z);
  return (uint8_t)(p - f->payload);
}

uint8_t build_gravity_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_float_32(p, s->bno085_gravity_x);
  p = pack_float_32(p, s->bno085_gravity_y);
  p = pack_float_32(p, s->bno085_gravity_z);
  return update_payload_length(f, start, p);
}

uint8_t parse_gravity_payload(const momentum_frame_t *f, sensor_data_t *s) {
  const uint8_t *p = f->payload;
  p = unpack_float_32(p, &s->bno085_gravity_x);
  p = unpack_float_32(p, &s->bno085_gravity_y);
  p = unpack_float_32(p, &s->bno085_gravity_z);
  return (uint8_t)(p - f->payload);
}

uint8_t build_pressure_temp_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_double_64(p, s->bmp390_temperature);
  p = pack_double_64(p, s->bmp390_pressure);
  return update_payload_length(f, start, p);
}

uint8_t parse_pressure_temp_payload(const momentum_frame_t *f,
                                    sensor_data_t *s) {
  const uint8_t *p = f->payload;
  p = unpack_double_64(p, &s->bmp390_temperature);
  p = unpack_double_64(p, &s->bmp390_pressure);
  return (uint8_t)(p - f->payload);
}

uint8_t build_gps_datetime_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_uint_8(p, s->gps_hour);
  p = pack_uint_8(p, s->gps_minute);
  p = pack_uint_8(p, s->gps_second);
  p = pack_uint_8(p, s->gps_day);
  p = pack_uint_8(p, s->gps_month);
  p = pack_uint_8(p, s->gps_year);
  return update_payload_length(f, start, p);
}

uint8_t parse_gps_datetime_payload(const momentum_frame_t *f,
                                   sensor_data_t *s) {
  const uint8_t *p = f->payload;
  p = unpack_uint_8(p, &s->gps_hour);
  p = unpack_uint_8(p, &s->gps_minute);
  p = unpack_uint_8(p, &s->gps_second);
  p = unpack_uint_8(p, &s->gps_day);
  p = unpack_uint_8(p, &s->gps_month);
  p = unpack_uint_8(p, &s->gps_year);
  return (uint8_t)(p - f->payload);
}

uint8_t build_gps_coord_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_double_64(p, s->gps_latitude);
  p = pack_char_8(p, s->gps_lat_dir);
  p = pack_double_64(p, s->gps_longitude);
  p = pack_char_8(p, s->gps_lon_dir);
  return update_payload_length(f, start, p);
}

uint8_t parse_gps_coord_payload(const momentum_frame_t *f, sensor_data_t *s) {
  const uint8_t *p = f->payload;
  p = unpack_double_64(p, &s->gps_latitude);
  p = unpack_char_8(p, &s->gps_lat_dir);
  p = unpack_double_64(p, &s->gps_longitude);
  p = unpack_char_8(p, &s->gps_lon_dir);
  return (uint8_t)(p - f->payload);
}

uint8_t build_gps_stats_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_uint_8(p, s->gps_fix_quality);
  p = pack_uint_8(p, s->gps_satellites);
  p = pack_float_32(p, s->gps_hdop);
  p = pack_double_64(p, s->gps_altitude);
  p = pack_double_64(p, s->gps_geoid_sep);
  return update_payload_length(f, start, p);
}

uint8_t parse_gps_stats_payload(const momentum_frame_t *f, sensor_data_t *s) {
  const uint8_t *p = f->payload;
  p = unpack_uint_8(p, &s->gps_fix_quality);
  p = unpack_uint_8(p, &s->gps_satellites);
  p = unpack_float_32(p, &s->gps_hdop);
  p = unpack_double_64(p, &s->gps_altitude);
  p = unpack_double_64(p, &s->gps_geoid_sep);
  return (uint8_t)(p - f->payload);
}

momentum_status_t parse_momentum_frame(const momentum_frame_t *f,
                                       sensor_data_t *s) {
  if (f->start_of_frame != MOMENTUM_START_OF_FRAME ||
      f->length > MOMENTUM_MAX_DATA_SIZE)
    return MOMENTUM_ERROR_BAD_FRAME;

  if (!verify_crc(f))
    return MOMENTUM_ERROR_CRC;

  switch (f->frame_type) {
  case MOMENTUM_FRAME_TYPE_IMU_QUAT:
    parse_quaternion_payload(f, s);
    break;
  case MOMENTUM_FRAME_TYPE_IMU_GYRO:
    parse_gyro_payload(f, s);
    break;
  case MOMENTUM_FRAME_TYPE_IMU_ACCEL:
    parse_accel_payload(f, s);
    break;
  case MOMENTUM_FRAME_TYPE_IMU_LINACCEL:
    parse_lin_accel_payload(f, s);
    break;
  case MOMENTUM_FRAME_TYPE_IMU_GRAV:
    parse_gravity_payload(f, s);
    break;
  case MOMENTUM_FRAME_TYPE_BAR_ENV:
    parse_pressure_temp_payload(f, s);
    break;
  case MOMENTUM_FRAME_TYPE_GPS_COORD:
    parse_gps_coord_payload(f, s);
    break;
  case MOMENTUM_FRAME_TYPE_GPS_DATETIME:
    parse_gps_datetime_payload(f, s);
    break;
  case MOMENTUM_FRAME_TYPE_GPS_STATS:
    parse_gps_stats_payload(f, s);
    break;
  default:
    return MOMENTUM_ERROR_FRAME_TYPE;
  }

  return MOMENTUM_OK; // Successful frame unpacking.
}
