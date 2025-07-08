/*******************************************************************************
 * @file momentum_runner.c
 * @brief Momentum runner handling low level interacting functions.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "momentum_runner.h"
#include "bmp390_runner.h"
#include "bno085_runner.h"
#include "momentum_driver.h"
#include "ublox_hal_uart.h"
#include "ws2812b_hal_pwm.h"
#include <string.h>

/** Private variables. ********************************************************/

// Receive variables.
static uint8_t rx_buffer[sizeof(momentum_frame_t)];
static momentum_frame_t rx_frame;

// Transmit variables.
static uint8_t tx_buffer[sizeof(momentum_frame_t)];
static volatile uint32_t tx_buf_len;

// Rolling sequence counter for outgoing frames.
static uint8_t seq_counter = 0;

// SPI communication state machine.
/**
 * @brief Momentum sensor hub SPI 2 phase communication state machine.
 *
 * Utilizes a 2 phase communication state:
 *  1. Controller request command.
 *  2. Peripheral response to request.
 */
typedef enum {
  WAIT_HEADER,   // Phase 1: Wait for frame header from controller.
  WAIT_PAYLOAD,  // Phase 2: Wait for remaining payload data.
  REACTIONARY,   // Phase 3.A: Complete frame reactionary logic.
  SEND_RESPONSE, // Phase 3.C: Transmit response frame.
} momentum_state_t;
static momentum_state_t spi_state = WAIT_HEADER;

/** Private functions. ********************************************************/

/**
 * @brief Prepare and pack a frame in response.
 */
void prep_momentum_response_tx(momentum_frame_t request) {
  // Assemble the latest sensor_data_t.
  sensor_data_t data = {bno085_quaternion_i,
                        bno085_quaternion_j,
                        bno085_quaternion_k,
                        bno085_quaternion_real,
                        bno085_quaternion_accuracy_rad,
                        bno085_quaternion_accuracy_deg,
                        bno085_gyro_x,
                        bno085_gyro_y,
                        bno085_gyro_z,
                        bno085_accel_x,
                        bno085_accel_y,
                        bno085_accel_z,
                        bno085_lin_accel_x,
                        bno085_lin_accel_y,
                        bno085_lin_accel_z,
                        bno085_gravity_x,
                        bno085_gravity_y,
                        bno085_gravity_z,
                        bmp390_temperature,
                        bmp390_pressure,
                        gps_data.position_fix,
                        gps_data.year,
                        gps_data.month,
                        gps_data.day,
                        gps_data.hour,
                        gps_data.minute,
                        gps_data.second,
                        gps_data.latitude,
                        gps_data.lat_dir,
                        gps_data.longitude,
                        gps_data.lon_dir,
                        gps_data.altitude_m,
                        gps_data.geoid_sep_m,
                        gps_data.speed_knots,
                        gps_data.course_deg,
                        gps_data.magnetic_deg,
                        gps_data.mag_dir,
                        gps_data.satellites,
                        gps_data.hdop};

  version_t version = {MOMENTUM_VERSION_MAJOR, MOMENTUM_VERSION_MINOR,
                       MOMENTUM_VERSION_PATCH, MOMENTUM_VERSION_IDENTIFIER};

  // Build the frame.
  momentum_frame_t frame;
  memset(&frame, 0, sizeof(frame));
  frame.start_of_frame = MOMENTUM_START_OF_RESPONSE_FRAME;
  frame.frame_type = request.frame_type;
  frame.sequence = seq_counter++;

  // Pack payload based on requested frame type.
  uint8_t len = 0;
  switch (frame.frame_type) {
  case MOMENTUM_FRAME_TYPE_VERSION:
    len = build_version_payload(&frame, &version);
    break;
  case MOMENTUM_FRAME_TYPE_IMU_QUAT:
    len = build_quaternion_payload(&frame, &data);
    break;
  case MOMENTUM_FRAME_TYPE_IMU_GYRO:
    len = build_gyro_payload(&frame, &data);
    break;
  case MOMENTUM_FRAME_TYPE_IMU_ACCEL:
    len = build_accel_payload(&frame, &data);
    break;
  case MOMENTUM_FRAME_TYPE_IMU_LINACCEL:
    len = build_lin_accel_payload(&frame, &data);
    break;
  case MOMENTUM_FRAME_TYPE_IMU_GRAV:
    len = build_gravity_payload(&frame, &data);
    break;
  case MOMENTUM_FRAME_TYPE_BAR_ENV:
    len = build_pressure_temp_payload(&frame, &data);
    break;
  case MOMENTUM_FRAME_TYPE_GPS_DATETIME:
    len = build_gps_datetime_payload(&frame, &data);
    break;
  case MOMENTUM_FRAME_TYPE_GPS_COORD:
    len = build_gps_coord_payload(&frame, &data);
    break;
  case MOMENTUM_FRAME_TYPE_GPS_ALT_SPEED:
    len = build_gps_altitude_speed_payload(&frame, &data);
    break;
  case MOMENTUM_FRAME_TYPE_GPS_HEAD:
    len = build_gps_heading_payload(&frame, &data);
    break;
  case MOMENTUM_FRAME_TYPE_GPS_STATS:
    len = build_gps_stats_payload(&frame, &data);
    break;
  default:
    len = 0;
    break;
  }

  // Update frame length and CRC.
  frame.length = len;
  build_crc(&frame);

  // Assemble frame header in the tx buffer.
  uint8_t idx = 0;
  tx_buffer[idx++] = frame.start_of_frame;
  tx_buffer[idx++] = frame.frame_type;
  tx_buffer[idx++] = frame.sequence;
  tx_buffer[idx++] = frame.length;

  // Copy exactly "length" bytes of payload.
  memcpy(&tx_buffer[idx], frame.payload, frame.length);
  idx += frame.length;

  // Append CRC (low byte, then high byte).
  tx_buffer[idx++] = (uint8_t)(frame.crc & 0xFF);
  tx_buffer[idx++] = (uint8_t)((frame.crc >> 8) & 0xFF);

  // Update tx buffer length variable.
  tx_buf_len = idx;
}

void process_header(void) {
  // Validate received frame header.
  rx_frame.start_of_frame = rx_buffer[0];
  rx_frame.frame_type = rx_buffer[1];
  rx_frame.sequence = rx_buffer[2];
  rx_frame.length = rx_buffer[3];

  if (rx_frame.start_of_frame != MOMENTUM_START_OF_COMMAND_FRAME &&
      rx_frame.start_of_frame != MOMENTUM_START_OF_REQUEST_FRAME) {
    // Start of frame error.
    spi_state = WAIT_HEADER; // Reset to waiting for new header.
    return;
  }

  // TODO: Implement rolling sequence counter checking.
}

void process_payload(uint8_t *rx_payload) {
  memcpy(rx_frame.payload, rx_payload, rx_frame.length); // Copy in payload.
  size_t crc_index = rx_frame.length;         // Determine CRC location.
  uint8_t crc_lo = rx_payload[crc_index + 0]; // CRC low byte.
  uint8_t crc_hi = rx_payload[crc_index + 1]; // CRC high byte.
  rx_frame.crc = (uint16_t)crc_lo | ((uint16_t)crc_hi << 8); // Reconstruct CRC.

  // Check CRC for bad frame.
  if (!verify_crc(&rx_frame)) {
    // CRC error.
    spi_state = WAIT_HEADER; // Reset to waiting for new header.
    return;
  }
}

void process_reaction(momentum_frame_t request) {
  switch (request.frame_type) {
  case MOMENTUM_FRAME_TYPE_LED:
    ws2812b_set_colour(request.payload[0], request.payload[1],
                       request.payload[2], request.payload[3]);
    ws2812b_update();
    break;
  default:
    break;
  }
}

void restart(void) {
  // Restart state machine and await new 4 byte frame header.
  spi_state = WAIT_HEADER;
  HAL_SPI_Receive_DMA(&MOMENTUM_HSPI, rx_buffer, 4);
}

/** User implementations of STM32 UART HAL (overwriting HAL). *****************/

void HAL_SPI_RxCpltCallback_momentum(SPI_HandleTypeDef *hspi) {
  if (hspi != &MOMENTUM_HSPI) {
    return;
  }

  if (spi_state == WAIT_HEADER) {
    process_header();
    if (rx_frame.length) { // Additional payload data is expected to arrive.
      spi_state = WAIT_PAYLOAD;
      // Receive payload and 2 CRC bytes.
      HAL_SPI_Receive_DMA(&MOMENTUM_HSPI, rx_buffer, rx_frame.length + 2);
    } else { // Immediately handle reactionary logic.
      spi_state = REACTIONARY;
    }
  } else if (spi_state == WAIT_PAYLOAD) {
    process_payload(rx_buffer);
    spi_state = REACTIONARY;
  }
  if (spi_state == REACTIONARY) {
    process_reaction(rx_frame);

    // Check if frame expects a response.
    if (rx_frame.start_of_frame == MOMENTUM_START_OF_REQUEST_FRAME) {
      spi_state = SEND_RESPONSE;
      prep_momentum_response_tx(rx_frame); // Prep response.
      HAL_SPI_Transmit_DMA(&MOMENTUM_HSPI, tx_buffer, tx_buf_len);
    } else {
      restart();
    }
  }
}

void HAL_SPI_TxCpltCallback_momentum(SPI_HandleTypeDef *hspi) {
  if (hspi != &MOMENTUM_HSPI || spi_state != SEND_RESPONSE)
    return;

  restart();
}

/** Public functions. *********************************************************/

void momentum_spi_start(void) { restart(); }
