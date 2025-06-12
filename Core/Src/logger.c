/*******************************************************************************
 * @file logger.c
 * @brief Momentum sensor data logging to NVM memory.
 *******************************************************************************/

/** Includes. *****************************************************************/

#include "logger.h"
#include "bmp390_runner.h"
#include "bno085_runner.h"
#include "comm.h"
#include "momentum_driver.h"
#include "ublox_hal_uart.h"
#include "w25qxx_hal_spi.h"

/** Definitions. **************************************************************/

#define NVM_START_ADDRESS 0x001000UL
#define NVM_REGION_SIZE (256UL * 1024UL) // W25Q128 = 16 Mbit = 2 MB(yte).

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
 * @brief Erase the current sector if not already erased.
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
 * @brief Write one payload to flash and bump the pointer.
 * @param buf pointer to the data_array.
 * @param len how many bytes to program.
 */
static void logger_write(const uint8_t *buf, uint8_t len) {
  if (!comm_write_enabled) {
    return;
  }

  uint16_t remaining = len;
  const uint8_t *p = buf;

  while (remaining) {
    // 1) Check number of bytes left in the current 256-byte page.
    uint32_t page_off = nvm_address & (W25Q_PAGE_SIZE - 1);
    uint32_t space_in_page = W25Q_PAGE_SIZE - page_off;
    uint16_t to_write = (remaining < space_in_page) ? remaining : space_in_page;

    // 2) Erase the sector if just crossed into a new sector.
    ensure_sector_erased();

    // 3) Program up to `to_write` bytes within the current page.
    if (w25q_page_program(p, nvm_address, to_write) != HAL_OK) {
      // TODO: error handling/retry logic.
      return;
    }

    // 4) Advance pointers and counters.
    p += to_write;
    nvm_address += to_write;
    remaining -= to_write;
  }

  // 5) Wrap around the circular buffer if the end of the region is reached.
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

void log_quaternion(void) {
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

void log_gyro(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_IMU_GYRO);
  p = pack_float_32(p, bno085_gyro_x);
  p = pack_float_32(p, bno085_gyro_y);
  p = pack_float_32(p, bno085_gyro_z);
  uint8_t len = (uint8_t)(p - start);
  logger_write(data_array, len);
}

void log_accel(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_IMU_ACCEL);
  p = pack_float_32(p, bno085_accel_x);
  p = pack_float_32(p, bno085_accel_y);
  p = pack_float_32(p, bno085_accel_z);
  uint8_t len = (uint8_t)(p - start);
  logger_write(data_array, len);
}

void log_lin_accel(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_IMU_LINACCEL);
  p = pack_float_32(p, bno085_lin_accel_x);
  p = pack_float_32(p, bno085_lin_accel_y);
  p = pack_float_32(p, bno085_lin_accel_z);
  uint8_t len = (uint8_t)(p - start);
  logger_write(data_array, len);
}

void log_gravity(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_IMU_GRAV);
  p = pack_float_32(p, bno085_gravity_x);
  p = pack_float_32(p, bno085_gravity_y);
  p = pack_float_32(p, bno085_gravity_z);
  uint8_t len = (uint8_t)(p - start);
  logger_write(data_array, len);
}

void log_pressure_temp(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_BAR_ENV);
  p = pack_float_32(p, bmp390_temperature);
  p = pack_float_32(p, bmp390_pressure);
  uint8_t len = (uint8_t)(p - start);
  logger_write(data_array, len);
}

void log_gps_datetime(void) {
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

void log_gps_coord(void) {
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

void log_gps_altitude_speed(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_GPS_ALT_SPEED);
  p = pack_float_32(p, gps_data.altitude_m);
  p = pack_float_32(p, gps_data.geoid_sep_m);
  p = pack_float_32(p, gps_data.speed_knots);
  uint8_t len = (uint8_t)(p - start);
  logger_write(data_array, len);
}

void log_gps_heading(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_GPS_HEAD);
  p = pack_float_32(p, gps_data.course_deg);
  p = pack_float_32(p, gps_data.magnetic_deg);
  p = pack_char_8(p, gps_data.mag_dir);
  uint8_t len = (uint8_t)(p - start);
  logger_write(data_array, len);
}

void log_gps_stats(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_GPS_STATS);
  p = pack_uint_8(p, gps_data.position_fix);
  p = pack_uint_8(p, gps_data.satellites);
  p = pack_float_32(p, gps_data.hdop);
  uint8_t len = (uint8_t)(p - start);
  logger_write(data_array, len);
}
