/*******************************************************************************
 * @file init.c
 * @brief Centralized init logic running in main.c.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "init.h"
#include "bmp390_runner.h"
#include "bno085_runner.h"
#include "can.h"
#include "comm.h"
#include "configuration.h"
#include "momentum_runner.h"
#include "scheduler.h"
#include "ublox_hal_uart.h"
#include "w25qxx_hal_spi.h"
#include "ws2812b_hal_pwm.h"

/** Private variables. ********************************************************/

/** Private functions. ********************************************************/

/** Public functions. *********************************************************/

void momentum_init(void) {
  // Low level peripherals.
  can_init();

  // On-board miscellaneous components.
  ws2812b_init();
  ws2812b_set_colour(0, 4, 1, 1);
  ws2812b_update();

  // W25Qxx flash.
#ifdef MOMENTUM_W25QXX_ENABLE
  if (w25q_init() != HAL_OK) {
    return;
  }
  uint8_t manuf = 0;
  uint8_t mem_type = 0;
  uint8_t capacity = 0;
  w25q_read_jedec(&manuf, &mem_type, &capacity);
  if (manuf != MOMENTUM_W25QXX_EXPECTED_MANUF &&
      mem_type != MOMENTUM_W25QXX_EXPECTED_MEM_TYPE &&
      capacity != MOMENTUM_W25QXX_EXPECTED_CAPACITY) {
    return;
  }
#endif

  // Sensors.
  ublox_init();
  bmp390_init();
  bno085_reset();
  bno085_init();

  // Scheduler.
  scheduler_init(); // Initialize scheduler.
  scheduler_add_task(bmp390_get_data, 10);

  // Momentum SPI interface.
#ifndef MOMENTUM_W25QXX_ENABLE
  // Momentum runner SPI communication start.
  momentum_spi_start();
#else
  // NVM communication via USB-to-UART(1) start.
  comm_init();
#endif

  // On-board miscellaneous components.
  ws2812b_set_colour(0, 2, 2, 0);
  ws2812b_update();
}
