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
#define TEST_ADDR 0x000000UL
  // 1) Initialize the flash
  if (w25q_init() != HAL_OK) {
    printf("Flash init failed!\n");
  }

  // 2) Prepare your data
  const char write_data[] = "Hello, W25QFlash!";
  uint32_t len = sizeof(write_data); // includes the '\0'

  // 3) Erase the 4 KB sector at TEST_ADDR
  if (w25q_sector_erase(TEST_ADDR) != HAL_OK) {
    printf("Sector erase failed!\n");
  }

  // 4) Program one page (max 256 B) at TEST_ADDR
  if (w25q_page_program((uint8_t *)write_data, TEST_ADDR, len) != HAL_OK) {
    printf("Page program failed!\n");
  }

  // 5) Read back into a buffer
  char read_data[sizeof(write_data)] = {0};
  if (w25q_read_data((uint8_t *)read_data, TEST_ADDR, len) != HAL_OK) {
    printf("Read failed!\n");
  }

  // 6) Verify / print
  if (memcmp(write_data, read_data, len) == 0) {
    printf("SUCCESS: Read \"%s\"\n", read_data);
  } else {
    printf("MISMATCH! Wrote \"%s\" but read \"%s\"\n", write_data, read_data);
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

#ifndef MOMENTUM_W25QXX_ENABLE
  // Momentum runner SPI communication start.
  momentum_spi_start();
#endif
}
