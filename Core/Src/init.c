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
#include "configuration.h"
#include "mcu_temp_hal_adc.h"
#include "momentum_runner.h"
#include "scheduler.h"
#include "stm32l4xx_hal_rng.h"
#include "telemetry.h"
#include "ublox_hal_uart.h"
#include "ws2812b_hal_pwm.h"

/** STM32 port and pin configs. ***********************************************/

extern RNG_HandleTypeDef hrng;

/** Private functions. ********************************************************/

/**
 * @brief Transmit state message.
 */
static void can_tx_state(void) {
  const can_message_t state_msg = mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_STATE];
  uint32_t state_sigs[2] = {0};
  // TODO: Hardcoded state.
  const float state_source_sigs[2] = {0, get_mcu_temp()};
  for (int i = 0; i < state_msg.signal_count; ++i) {
    state_sigs[i] =
        physical_to_raw(state_source_sigs[i], &state_msg.signals[i]);
  }
  can_send_message_raw32(&hcan1, &state_msg, state_sigs);

  mcu_temp_start(); // Trigger next temperature read.
}

/**
 * @brief Transmit cached GNSS telemetry, staggered to ease CAN TX pressure.
 *
 * Scheduled on a 25 ms base tick. Each message is sent on its own tick so a
 * single invocation never queues all frame at once:
 *   - Phase 0 ( 0 ms): GNSS1 (latitude/longitude).
 *   - Phase 1 (25 ms): GNSS2 (speed/course/fix/satellites/HDOP).
 *   - Phase 2 (50 ms): GNSS3 (altitude/geoid separation).
 *   - Phase 3 (75 ms): idle (leaves a quiet bus slot).
 */
static void can_tx_gnss(void) {
  static uint8_t phase = 0;

#ifdef MOMENTUM_FULL_CAN_TELEMETRY
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  switch (phase) {
  case 0:
    can_tx_gnss1();
    break;
  case 1:
    can_tx_gnss2();
    break;
  case 2:
    can_tx_gnss3();
    break;
  default:
    break; // Idle tick.
  }

  // Restore the prior interrupt state (do not force-enable if already masked).
  if (!primask) {
    __enable_irq();
  }
#endif
#ifdef MOMENTUM_FULL_COMM_TELEMETRY
  switch (phase) {
  case 0:
    comm_tx_gnss1();
    break;
  case 1:
    comm_tx_gnss2();
    break;
  case 2:
    comm_tx_gnss3();
    break;
  default:
    break; // Idle tick.
  }
#endif

  phase = (uint8_t)((phase + 1u) & 0x03u); // Cycle 0..3 (100 ms period).
}

/**
 * @brief Drive the on-board WS2812B to reflect GNSS fix status.
 */
static void led_status_run(void) {
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

  // MCU internal core temperature sense (ADC self-calibration), then kick the
  // first DMA conversion so a reading is cached before first task trigger.
  mcu_temp_init();
  mcu_temp_start();

  // Scheduler.
  scheduler_init(); // Initialize scheduler.
  scheduler_add_task(can_tx_gnss, 25);
  scheduler_add_task(led_status_run, 100);
  scheduler_add_task(can_id_allocatee_state_machine, 250);
  scheduler_add_task(can_tx_state, 1000);
#if MOMENTUM_BAROMETER_REPORT_MS > 0
  scheduler_add_task(bmp390_get_data, MOMENTUM_BAROMETER_REPORT_MS);
#endif

  // Sensors.
  // TODO: DEV NOTE: Timer initialization completed by scheduler (TIM owner).
  //  Sensors need to be initialized after scheduler initialization.
  ublox_init();
  bmp390_init();
  bno085_reset();
  bno085_init();
}
