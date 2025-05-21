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

/** Private variables. ********************************************************/

// Receive support.
static uint8_t rx_buffer[MOMENTUM_MAX_FRAME_SIZE];
static volatile uint32_t rx_buf_len;

// Transmit support.
static uint8_t tx_buffer[MOMENTUM_MAX_FRAME_SIZE];
static uint32_t tx_buf_len;

/** Private types. ************************************************************/

typedef enum { WAIT_CMD, SEND_DATA } momentum_state_t;

static momentum_state_t spi_state = WAIT_CMD;

/** Private functions. ********************************************************/

void prep_momentum_tx(SPI_HandleTypeDef *hspi) {
  if (hspi == &MOMENTUM_HSPI) {
    if (rx_buffer[0] == 1) { // Data transmit request.
      sensor_data_t data = {
          bno085_quaternion_i,
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
          0, // TODO: Extract gps_hour from ublox NEMA RTC sentence.
          0, // TODO: Extract gps_minute from ublox NEMA RTC sentence.
          0, // TODO: Extract gps_second from ublox NEMA RTC sentence.
          0, // TODO: Extract gps_day from ublox NEMA RTC sentence.
          0, // TODO: Extract gps_month from ublox NEMA RTC sentence.
          0, // TODO: Extract gps_year from ublox NEMA RTC sentence.
          gps_data.latitude,
          gps_data.lat_dir,
          gps_data.longitude,
          gps_data.lon_dir,
          gps_data.fix_quality,
          gps_data.satellites,
          gps_data.hdop,
          gps_data.altitude,
          gps_data.geoid_sep,
      };

      // Pack sensor data for transmission.
      pack_sensor_data_8bit(tx_buffer, data);
    }
  }
}

/** Public functions. *********************************************************/

void HAL_SPI_TxCpltCallback_momentum(SPI_HandleTypeDef *hspi) {
  if (hspi == &MOMENTUM_HSPI) {
    if (spi_state == SEND_DATA) {
      spi_state = WAIT_CMD;
      HAL_SPI_Receive_DMA(&MOMENTUM_HSPI, rx_buffer, rx_buf_len);
    }
  }
}

void HAL_SPI_RxCpltCallback_momentum(SPI_HandleTypeDef *hspi) {
  if (hspi == &MOMENTUM_HSPI) {
    if (spi_state == WAIT_CMD && rx_buffer[0] == 1) {
      spi_state = SEND_DATA;
      // prepare_tx(); // TODO: Implement tx buffer loading.
      HAL_SPI_Transmit_DMA(&MOMENTUM_HSPI, tx_buffer, sizeof(tx_buffer));
    }
  }
}

void momentum_spi_start(void) {
  uint8_t dummy_tx[1] = {0};

  // Send dummy SPI transmission and await Rx messages.
  HAL_SPI_TransmitReceive_DMA(&MOMENTUM_HSPI, dummy_tx, rx_buffer,
                              sizeof(dummy_tx));
}
