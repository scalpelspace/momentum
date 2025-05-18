/*******************************************************************************
 * @file momentum_runner.c
 * @brief Momentum runner handling low level interacting functions.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "momentum_runner.h"
#include "momentum_driver.h"

/** Private variables. ********************************************************/

// Receive support.
static uint8_t rx_buffer[255];
static volatile uint32_t rx_buf_len;

// Transmit support.
static uint8_t tx_buffer[255];
static uint32_t tx_buf_len;

/** Public functions. *********************************************************/

void HAL_SPI_TxRxCpltCallback_momentum(SPI_HandleTypeDef *hspi) {
  if (hspi == &MOMENTUM_HSPI) {
    sensor_data_t data = {0};

    if (rx_buffer[0] == 1) { // Data transmit request.
      // Pack sensor data for transmission.
      pack_sensor_data_8bit(tx_buffer, data);

      // Transmit data.
      HAL_SPI_TransmitReceive_DMA(&MOMENTUM_HSPI, tx_buffer, rx_buffer, 8);
    }
  }
}
