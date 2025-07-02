/*******************************************************************************
 * @file run.c
 * @brief Centralized main loop run logic running in main.c.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "run.h"
#include "bno085_runner.h"
#include "comm.h"

/** Public functions. *********************************************************/

void momentum_run(void) {
#ifdef MOMENTUM_COMM_ENABLE
  comm_process_rx_data(); // UART1 communications RX data process.
#endif
  bno085_run();    // BNO085 process.
  scheduler_run(); // Run the scheduler.
}
