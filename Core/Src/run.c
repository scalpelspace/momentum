/*******************************************************************************
 * @file run.c
 * @brief Centralized main loop run logic running in main.c.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "run.h"
#include "bno085_runner.h"

/** Public functions. *********************************************************/

void momentum_run(void) {
  bno085_run();    // BNO085 process.
  scheduler_run(); // Run the scheduler.
}
