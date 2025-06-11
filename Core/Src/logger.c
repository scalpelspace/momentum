/*******************************************************************************
 * @file logger.c
 * @brief Momentum sensor data logging to NVM memory.
 *******************************************************************************/

/** Includes. *****************************************************************/

#include "logger.h"
#include "bmp390_runner.h"
#include "bno085_runner.h"
#include "momentum_driver.h"
#include "ublox_hal_uart.h"
#include "w25qxx_hal_spi.h"

/** Definitions. **************************************************************/

#define NVM_START_ADDRESS 0x001000UL
#define NVM_REGION_SIZE (256UL * 1024UL)
// W25Q128 = 16 Mbit = 2 M-byte; adjust if using subset
#define NVM_END_ADDRESS (NVM_START_ADDRESS + NVM_REGION_SIZE)

/** Private variables. ********************************************************/

static uint8_t data_array[32];
static uint32_t nvm_address;
static bool sector_erased;

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

/**
 * @brief Internal: erase the current sector if not already erased.
 */
static void ensure_sector_erased(void) {
  uint32_t sector_start = nvm_address & ~(W25Q_SECTOR_SIZE - 1);
  if (!sector_erased ||
      sector_start != ((nvm_address - 1) & ~(W25Q_SECTOR_SIZE - 1))) {
    w25q_sector_erase(sector_start);
    sector_erased = true;
  }
}

/**
 * @brief Internal: write one payload to flash and bump the pointer.
 * @param buf pointer to the data_array
 * @param len how many bytes to program (≤ 32 in your design)
 */
static void logger_write(const uint8_t *buf, uint8_t len) {
  // 1) Erase sector on first entry.
  ensure_sector_erased();

  // 2) Program the page.
  w25q_page_program((uint8_t *)buf, nvm_address, len);

  // 3) Advance pointer.
  nvm_address += len;

  // 4) Wrap around.
  if (nvm_address + W25Q_PAGE_SIZE > NVM_END_ADDRESS) {
    nvm_address = NVM_START_ADDRESS;
    sector_erased = false;
  }
}

/** Public functions. *********************************************************/

void logger_init(void) {
  nvm_address = NVM_START_ADDRESS;
  sector_erased = false;
}

void logger_reset(void) {
  for (uint32_t addr = NVM_START_ADDRESS; addr < NVM_END_ADDRESS;
       addr += W25Q_SECTOR_SIZE) {
    w25q_sector_erase(addr);
  }
  nvm_address = NVM_START_ADDRESS;
  sector_erased = true;
}

void log_quaternion_payload(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_IMU_QUAT);
  p = pack_float_32(p, bno085_quaternion_i);
  p = pack_float_32(p, bno085_quaternion_j);
  p = pack_float_32(p, bno085_quaternion_k);
  p = pack_float_32(p, bno085_quaternion_real);
  p = pack_float_32(p, bno085_quaternion_accuracy_rad);
  p = pack_float_32(p, bno085_quaternion_accuracy_deg);
  uint8_t len = (uint8_t)(p - start);
  logger_write(data_array, len);
}

void log_gyro_payload(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_IMU_GYRO);
  p = pack_float_32(p, bno085_gyro_x);
  p = pack_float_32(p, bno085_gyro_y);
  p = pack_float_32(p, bno085_gyro_z);
  uint8_t len = (uint8_t)(p - start);
  logger_write(data_array, len);
}

void log_accel_payload(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_IMU_ACCEL);
  p = pack_float_32(p, bno085_accel_x);
  p = pack_float_32(p, bno085_accel_y);
  p = pack_float_32(p, bno085_accel_z);
  uint8_t len = (uint8_t)(p - start);
  logger_write(data_array, len);
}

void log_lin_accel_payload(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_IMU_LINACCEL);
  p = pack_float_32(p, bno085_lin_accel_x);
  p = pack_float_32(p, bno085_lin_accel_y);
  p = pack_float_32(p, bno085_lin_accel_z);
  uint8_t len = (uint8_t)(p - start);
  logger_write(data_array, len);
}

void log_gravity_payload(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_IMU_GRAV);
  p = pack_float_32(p, bno085_gravity_x);
  p = pack_float_32(p, bno085_gravity_y);
  p = pack_float_32(p, bno085_gravity_z);
  uint8_t len = (uint8_t)(p - start);
  logger_write(data_array, len);
}

void log_pressure_temp_payload(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_BAR_ENV);
  p = pack_float_32(p, bmp390_temperature);
  p = pack_float_32(p, bmp390_pressure);
  uint8_t len = (uint8_t)(p - start);
  logger_write(data_array, len);
}

void log_gps_datetime_payload(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_GPS_DATETIME);
  p = pack_uint_8(p, gps_data.hour);
  p = pack_uint_8(p, gps_data.minute);
  p = pack_uint_8(p, gps_data.second);
  p = pack_uint_8(p, gps_data.day);
  p = pack_uint_8(p, gps_data.month);
  p = pack_uint_8(p, gps_data.year);
  uint8_t len = (uint8_t)(p - start);
  logger_write(data_array, len);
}

void log_gps_coord_payload(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_GPS_COORD);
  p = pack_float_32(p, gps_data.latitude);
  p = pack_char_8(p, gps_data.lat_dir);
  p = pack_float_32(p, gps_data.longitude);
  p = pack_char_8(p, gps_data.lon_dir);
  uint8_t len = (uint8_t)(p - start);
  logger_write(data_array, len);
}

void log_gps_altitude_speed_payload(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_GPS_ALT_SPEED);
  p = pack_float_32(p, gps_data.altitude_m);
  p = pack_float_32(p, gps_data.geoid_sep_m);
  p = pack_float_32(p, gps_data.speed_knots);
  uint8_t len = (uint8_t)(p - start);
  logger_write(data_array, len);
}

void log_gps_heading_payload(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_GPS_HEAD);
  p = pack_float_32(p, gps_data.course_deg);
  p = pack_float_32(p, gps_data.magnetic_deg);
  p = pack_char_8(p, gps_data.mag_dir);
  uint8_t len = (uint8_t)(p - start);
  logger_write(data_array, len);
}

void log_gps_stats_payload(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_GPS_STATS);
  p = pack_uint_8(p, gps_data.position_fix);
  p = pack_uint_8(p, gps_data.satellites);
  p = pack_float_32(p, gps_data.hdop);
  uint8_t len = (uint8_t)(p - start);
  logger_write(data_array, len);
}
