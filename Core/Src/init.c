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
#include "momentum_runner.h"
#include "scheduler.h"
#include "stm32l4xx_hal_rng.h"
#include "ublox_hal_uart.h"
#include "ws2812b_hal_pwm.h"

/** STM32 port and pin configs. ***********************************************/

extern RNG_HandleTypeDef hrng;

/** Private variables. ********************************************************/

/** Private functions. ********************************************************/

/** Public functions. *********************************************************/

void momentum_init(void) {
  // Low level peripherals.
  can_init();

  // On-board miscellaneous components.
  ws2812b_init();
  ws2812b_set_colour(0, 2, 1, 3); // Init colour.
  ws2812b_update();

  // Momentum runner SPI communication start.
  momentum_spi_start();

  // Scheduler.
  scheduler_init(); // Initialize scheduler.
  scheduler_add_task(bmp390_get_data, 10);

  // Sensors.
  // TODO: DEV NOTE: Timer initialization completed by scheduler (TIM owner).
  //  Sensors need to be initialized after scheduler initialization.
  ublox_init();
  bmp390_init();
  bno085_reset();
  bno085_init();
}
