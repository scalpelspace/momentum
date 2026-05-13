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
#include "can_id_allocatee.h"
#include "comm.h"
#include "momentum_runner.h"
#include "scheduler.h"
#include "stm32l4xx_hal_rng.h"
#include "ublox_hal_uart.h"
#include "ws2812b_hal_pwm.h"

/** STM32 port and pin configs. ***********************************************/

extern RNG_HandleTypeDef hrng;

/** Private functions. ********************************************************/

/**
 * @brief Drive the on-board WS2812B to reflect GNSS fix status.
 */
void led_status_run(void) {
  static nmea_position_fix_t last_fix = FIX_TYPE_UNDETERMINED;
  const nmea_position_fix_t fix = gnss_data.position_fix;

  if (fix == last_fix)
    return;
  last_fix = fix;

  switch (fix) {
  case FIX_TYPE_GNSS_FIX:
  case FIX_TYPE_DR_FIX:
  case FIX_TYPE_RTK_FLOAT:
  case FIX_TYPE_RTK_FIX:
    ws2812b_set_colour(0, 0, 5, 0); // Green: GNSS lock acquired.
    break;
  case FIX_TYPE_NO_FIX:
  case FIX_TYPE_GNSS_LIMITS_EXCEEDED:
  case FIX_TYPE_DR_LIMITS_EXCEEDED:
    ws2812b_set_colour(0, 3, 3, 0); // Yellow: awaiting fix.
    break;
  case FIX_TYPE_UNDETERMINED:
  default:
    ws2812b_set_colour(0, 2, 1, 3); // Idle: dim init colour.
    break;
  }
  ws2812b_update();
}

/** Public functions. *********************************************************/

void momentum_init(void) {
  // Low level peripherals.
  can_init();
  can_db_init();

  // UART debug/developer interface.
  comm_init();

  // On-board miscellaneous components.
  ws2812b_init();
  ws2812b_set_colour(0, 2, 1, 3); // Init colour.
  ws2812b_update();

  // Momentum runner SPI communication start.
  momentum_spi_start();

  // CAN allocatee begin (configuration check internally).
  auto_can_id_allocatee_start();

  // Scheduler.
  scheduler_init(); // Initialize scheduler.
  scheduler_add_task(bmp390_get_data, 10);
  scheduler_add_task(can_id_allocatee_state_machine, 20);
  scheduler_add_task(led_status_run, 100);

  // Sensors.
  // TODO: DEV NOTE: Timer initialization completed by scheduler (TIM owner).
  //  Sensors need to be initialized after scheduler initialization.
  ublox_init();
  bmp390_init();
  bno085_reset();
  bno085_init();
}
