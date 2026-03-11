/*******************************************************************************
 * @file run.c
 * @brief Centralized main loop run logic running in main.c.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "run.h"
#include "bno085_runner.h"
#include "can_id_allocatee.h"

/** Public functions. *********************************************************/

void momentum_run(void) {
  bno085_run(); // BNO085 process.

  // TODO: Consider some kind of conditional flagged operation to reduce
  //  overhead.
  can_id_allocatee_state_machine(); // CAN ID allocatee state machine.

  scheduler_run(); // Run the scheduler.
}
