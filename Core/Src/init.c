/*******************************************************************************
 * @file init.c
 * @brief Centralized init logic running in main.c.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "init.h"
#include "bmp390_runner.h"
#include "bno085_runner.h"
#include "scheduler.h"
#include "ublox_hal_uart.h"
#include "ws2812b_hal_pwm.h"

/** Private variables. ********************************************************/

/** Private functions. ********************************************************/

/** Public functions. *********************************************************/

void momentum_init(void) {
  // On-board miscellaneous components.
  ws2812b_init();
  ws2812b_set_colour(0, 4, 1, 1);
  ws2812b_update();

  // Sensors.
  ublox_init();
  bmp390_init();
  bno085_reset();
  bno085_init();

  // Scheduler.
  scheduler_init(); // Initialize scheduler.
  scheduler_add_task(bmp390_get_data, 10);
}
