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
#define NVM_REGION_SIZE (256UL * 1024UL) // W25Q128 = 16 Mbit = 2 MB(yte).

#define NVM_END_ADDRESS (NVM_START_ADDRESS + NVM_REGION_SIZE)

/** Private variables. ********************************************************/

static uint8_t data_array[32];

// Region bounds & state
static uint32_t nvm_start;
static uint32_t nvm_end;
static uint32_t session_id;
static uint32_t page_seq;
static uint32_t current_page_addr;
static uint16_t page_offset;

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

// Helpers: check if the header area of a page is fully erased
static bool is_page_blank(uint32_t addr) {
  uint8_t buf[sizeof(log_page_header_t)];
  if (w25q_read_data(buf, addr, sizeof(buf)) != HAL_OK) {
    return false;
  }
  for (uint16_t i = 0; i < sizeof(buf); i++) {
    if (buf[i] != 0xFFU)
      return false;
  }
  return true;
}

// Find the next blank page at or after from_addr (wrapping to start).
static uint32_t find_next_blank_page(uint32_t from_addr) {
  // 1) forward scan
  for (uint32_t a = from_addr; a + W25Q_PAGE_SIZE <= nvm_end;
       a += W25Q_PAGE_SIZE) {
    if (is_page_blank(a))
      return a;
  }
  // 2) wrap-around
  for (uint32_t a = nvm_start; a + W25Q_PAGE_SIZE <= from_addr;
       a += W25Q_PAGE_SIZE) {
    if (is_page_blank(a))
      return a;
  }
  return INVALID_ADDRESS;
}

/** Public functions. *********************************************************/

void logger_init(uint32_t start_address, uint32_t end_address,
                 uint32_t sess_id) {
  nvm_start = start_address;
  nvm_end = end_address;
  session_id = sess_id;
  page_seq = 0;
  page_offset = sizeof(log_page_header_t);
  // Pick up where the program left off (first blank page).
  current_page_addr = find_next_blank_page(nvm_start);
}

void logger_hard_reset(void) {
  // Erase every sector in the region.
  for (uint32_t addr = nvm_start; addr < nvm_end; addr += W25Q_SECTOR_SIZE) {
    w25q_sector_erase(addr);
  }
  // Reset counters.
  page_seq = 0;
  page_offset = sizeof(log_page_header_t);
  current_page_addr = nvm_start;
}

void logger_write(const uint8_t *buf, uint16_t len) {
  uint16_t remaining = len;

  while (remaining && current_page_addr != INVALID_ADDRESS) {
    // 1) At the start of a new page write a header.
    if (page_offset == sizeof(log_page_header_t)) {
      // Skip if a header already exists.
      if (!is_page_blank(current_page_addr)) {
        current_page_addr =
            find_next_blank_page(current_page_addr + W25Q_PAGE_SIZE);
        page_offset = sizeof(log_page_header_t);
        continue;
      }
      log_page_header_t hdr = {.magic = LOG_MAGIC,
                               .session_id = session_id,
                               .page_seq = page_seq++};
      if (w25q_page_program((uint8_t *)&hdr, current_page_addr, sizeof(hdr)) !=
          HAL_OK) {
        // Skip whole page on error.
        current_page_addr =
            find_next_blank_page(current_page_addr + W25Q_PAGE_SIZE);
        page_offset = sizeof(log_page_header_t);
        continue;
      }
    }

    // 2) Calculate how much can be written into this page.
    uint16_t space = W25Q_PAGE_SIZE - page_offset;
    uint16_t to_write = (remaining < space) ? remaining : space;
    uint32_t addr = current_page_addr + page_offset;

    // 3) Program the data chunk.
    if (w25q_page_program(buf, addr, to_write) != HAL_OK) {
      // Skip whole page on error.
      current_page_addr =
          find_next_blank_page(current_page_addr + W25Q_PAGE_SIZE);
      page_offset = sizeof(log_page_header_t);
      continue;
    }

    // 4) Advance pointers.
    buf += to_write;
    remaining -= to_write;
    page_offset += to_write;

    // 5) If page is full, move to next blank page.
    if (page_offset >= W25Q_PAGE_SIZE) {
      current_page_addr =
          find_next_blank_page(current_page_addr + W25Q_PAGE_SIZE);
      page_offset = sizeof(log_page_header_t);
    }
  }
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
