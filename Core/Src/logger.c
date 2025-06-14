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

// Region bounds & state.
static uint32_t nvm_start;
static uint32_t nvm_end;
static uint32_t session_id;
static uint32_t page_seq;

// Ring buffer for combined single write.
static uint8_t page_buf[W25Q_PAGE_SIZE];
static uint16_t buf_off;       // Next free byte in page_buf[].
static uint32_t buf_page_addr; // Flash page this buf maps to.

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
 * @brief Check if the header area of a page is fully erased.
 *
 * @param addr Address of buffer to check.
 *
 * @return True if the given page of the address is free for a header write.
 */
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

/**
 * @brief Find the next blank page at or after from_addr (wrapping to start).
 */
static uint32_t find_next_blank_page(uint32_t from_addr) {
  // 1) Forward scan.
  for (uint32_t a = from_addr; a + W25Q_PAGE_SIZE <= nvm_end;
       a += W25Q_PAGE_SIZE) {
    if (is_page_blank(a))
      return a;
  }
  // 2) Wrap-around.
  for (uint32_t a = nvm_start; a + W25Q_PAGE_SIZE <= from_addr;
       a += W25Q_PAGE_SIZE) {
    if (is_page_blank(a))
      return a;
  }
  return INVALID_ADDRESS;
}

/**
 * @brief Write the ring buffer (page_buf) to NVM.
 */
static void logger_flush_page(void) {
  if (buf_off == 0) {
    return;
  }
  // Flush the ring buffer to program one page simultaneously.
  //  w25q_wait_busy();
  w25q_page_program(page_buf, buf_page_addr, W25Q_PAGE_SIZE);
  buf_off = 0;
}

/**
 * @brief Find and claim the next blank page, prepare buf_off and page_buf.
 */
static void allocate_next_page(void) {
  // 1) If this isn't the first call, move past the last page claimed.
  if (buf_page_addr != INVALID_ADDRESS) {
    buf_page_addr = find_next_blank_page(buf_page_addr + W25Q_PAGE_SIZE);
  } else {
    // First-time: start at beginning of region.
    buf_page_addr = find_next_blank_page(nvm_start);
  }

  // 2) If there are no blank pages, mark invalid.
  if (buf_page_addr == INVALID_ADDRESS) {
    return;
  }

  // 3) Reset the ring buffer offset to leave room for the header.
  buf_off = sizeof(log_page_header_t);

  // 4) Pre-fill buffer with 0xFF so you can inspect partial pages.
  memset(page_buf, 0xFF, W25Q_PAGE_SIZE);
}

/**
 * @brief Add new data to the ring buffer and manage a potentially full buffer.
 *
 * @param data Data to write into the ring buffer.
 * @param len Length of data in bytes.
 */
static void logger_buffered_write(const uint8_t *data, uint16_t len) {
  // If the new data won't fit, flush current page first.
  if (buf_off + len > W25Q_PAGE_SIZE) {
    logger_flush_page();
    allocate_next_page();
  }
  // Copy new log into RAM.
  memcpy(&page_buf[buf_off], data, len);
  buf_off += len;
}

/** Public functions. *********************************************************/

void logger_init(uint32_t start_address, uint32_t end_address,
                 uint32_t sess_id) {
  // 1) Region & session.
  nvm_start = start_address;
  nvm_end = end_address;
  session_id = sess_id;

  // 2) Sequence and buffer state.
  page_seq = 0;
  buf_page_addr = INVALID_ADDRESS;

  // 3) Grab our first blank page and prep the RAM buffer.
  allocate_next_page();
}

void logger_hard_reset(void) {
  // 1) Erase every sector in the region (and wait until finish.)
  for (uint32_t addr = nvm_start; addr < nvm_end; addr += W25Q_SECTOR_SIZE) {
    w25q_sector_erase(addr);
    w25q_wait_busy();
  }

  // 2) Reset in-RAM buffer state.
  page_seq = 0;
  buf_page_addr = INVALID_ADDRESS;
  allocate_next_page();
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
  logger_buffered_write(data_array, len);
}

void log_gyro(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_IMU_GYRO);
  p = pack_float_32(p, bno085_gyro_x);
  p = pack_float_32(p, bno085_gyro_y);
  p = pack_float_32(p, bno085_gyro_z);
  uint8_t len = (uint8_t)(p - start);
  logger_buffered_write(data_array, len);
}

void log_accel(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_IMU_ACCEL);
  p = pack_float_32(p, bno085_accel_x);
  p = pack_float_32(p, bno085_accel_y);
  p = pack_float_32(p, bno085_accel_z);
  uint8_t len = (uint8_t)(p - start);
  logger_buffered_write(data_array, len);
}

void log_lin_accel(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_IMU_LINACCEL);
  p = pack_float_32(p, bno085_lin_accel_x);
  p = pack_float_32(p, bno085_lin_accel_y);
  p = pack_float_32(p, bno085_lin_accel_z);
  uint8_t len = (uint8_t)(p - start);
  logger_buffered_write(data_array, len);
}

void log_gravity(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_IMU_GRAV);
  p = pack_float_32(p, bno085_gravity_x);
  p = pack_float_32(p, bno085_gravity_y);
  p = pack_float_32(p, bno085_gravity_z);
  uint8_t len = (uint8_t)(p - start);
  logger_buffered_write(data_array, len);
}

void log_pressure_temp(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_BAR_ENV);
  p = pack_float_32(p, bmp390_temperature);
  p = pack_float_32(p, bmp390_pressure);
  uint8_t len = (uint8_t)(p - start);
  logger_buffered_write(data_array, len);
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
  logger_buffered_write(data_array, len);
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
  logger_buffered_write(data_array, len);
}

void log_gps_altitude_speed(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_GPS_ALT_SPEED);
  p = pack_float_32(p, gps_data.altitude_m);
  p = pack_float_32(p, gps_data.geoid_sep_m);
  p = pack_float_32(p, gps_data.speed_knots);
  uint8_t len = (uint8_t)(p - start);
  logger_buffered_write(data_array, len);
}

void log_gps_heading(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_GPS_HEAD);
  p = pack_float_32(p, gps_data.course_deg);
  p = pack_float_32(p, gps_data.magnetic_deg);
  p = pack_char_8(p, gps_data.mag_dir);
  uint8_t len = (uint8_t)(p - start);
  logger_buffered_write(data_array, len);
}

void log_gps_stats(void) {
  uint8_t *start = data_array;
  uint8_t *p = data_array;
  p = pack_uint_8(p, MOMENTUM_FRAME_TYPE_GPS_STATS);
  p = pack_uint_8(p, gps_data.position_fix);
  p = pack_uint_8(p, gps_data.satellites);
  p = pack_float_32(p, gps_data.hdop);
  uint8_t len = (uint8_t)(p - start);
  logger_buffered_write(data_array, len);
}
