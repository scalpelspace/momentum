/*******************************************************************************
 * @file scheduler.c
 * @brief Scheduler: Manages real time scheduling via DWT.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "scheduler.h"
#include "core_cm4.h" // Include core definitions for DWT.

/** Definitions. **************************************************************/

#define CPU_CYCLES_PER_MS (SystemCoreClock / 1000U)

/** Private variables. ********************************************************/

static task_t tasks[MAX_TASKS];
static uint8_t num_tasks = 0;

/** Public functions. *********************************************************/

void scheduler_init(void) {
  // Enable DWT and the cycle counter.
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }

  // Enable the cycle counter.
  if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }

  // Reset the cycle counter.
  DWT->CYCCNT = 0;
}

void scheduler_add_task(task_function_t task_function, uint32_t period_ms) {
  __disable_irq();

  // Add task.
  if (num_tasks < MAX_TASKS) {
    tasks[num_tasks].task_function = task_function;
    tasks[num_tasks].period_cyc = period_ms * CPU_CYCLES_PER_MS;
    tasks[num_tasks].next_execution_cyc =
        DWT->CYCCNT + tasks[num_tasks].period_cyc;
    num_tasks++;
  } else {
    // Handle error: Maximum number of tasks reached.
  }

  __enable_irq();
}

void scheduler_run(void) {
  uint32_t current_cyc = DWT->CYCCNT;

  for (uint8_t i = 0; i < num_tasks; i++) {
    if ((int32_t)(current_cyc - tasks[i].next_execution_cyc) >= 0) {
      tasks[i].task_function();
      tasks[i].next_execution_cyc += tasks[i].period_cyc;
    }
  }
}
