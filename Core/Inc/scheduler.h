/*******************************************************************************
 * @file scheduler.h
 * @brief Scheduler: Manages real time scheduling via DWT.
 *******************************************************************************
 */

#ifndef MOMENTUM__SCHEDULER_H
#define MOMENTUM__SCHEDULER_H

/** Includes. *****************************************************************/

#include "stm32l4xx_hal.h"

/** Definitions. **************************************************************/

// Ensure DWT utilization is possible.
#if defined(DWT)
// Init exists.
#else
#warning "DWT not available, scheduler timing disabled."
#endif

#define MAX_TASKS 10

/** Public types. *************************************************************/

/**
 * @brief Defined type for task functions.
 */
typedef void (*task_function_t)(void);

/**
 * @brief Structure to hold task information.
 *
 * task_function: A function pointer to the task that needs to be executed.
 * period_cyc: The period of the task in terms of CPU cycles.
 *  This is calculated by converting the desired period in milliseconds to CPU
 *  cycles using the formula period_cyc = period_ms * CPU_CYCLES_PER_MS.
 * next_execution_cyc: The absolute CPU cycle count at which the task is next
 *  scheduled to run. This is used to determine when the task should be executed
 *  based on the DWT cycle counter.
 */
typedef struct {
  task_function_t task_function; // Pointer to the task function.
  uint32_t period_cyc;           // Task execution period in CPU cycles.
  uint32_t next_execution_cyc;   // Next execution time in CPU cycles.
} task_t;

/** Public functions. *********************************************************/

/**
 * @brief Initialize scheduler which utilized DWT.
 */
void scheduler_init(void);

/**
 * Function to add tasks to the scheduler.
 *
 * @param task_function task_function_t to add as a task.
 * @param period_ms Task execution period in milliseconds.
 */
void scheduler_add_task(task_function_t task_function, uint32_t period_ms);

/**
 * @brief Scheduler run function to be called in the main loop.
 */
void scheduler_run(void);

#endif
