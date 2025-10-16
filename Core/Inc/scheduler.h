/*******************************************************************************
 * @file scheduler.h
 * @brief Scheduler: Manages real time scheduling via timer peripheral.
 *******************************************************************************
 */

#ifndef MOMENTUM__SCHEDULER_H
#define MOMENTUM__SCHEDULER_H

/** Includes. *****************************************************************/

#include "stm32l4xx_hal.h"

/** STM32 port and pin configs. ***********************************************/

extern TIM_HandleTypeDef htim2;

// Timer for signals.
#define SCHEDULER_HTIM htim2

// Timer address.
#define SCHEDULER_TIM TIM2

/** Definitions. **************************************************************/

#define MAX_TASKS 10

/** Public types. *************************************************************/

/**
 * @brief Defined type for task functions.
 */
typedef void (*task_function_t)(void);

/**
 * @brief Structure to hold task information.
 */
typedef struct {
  task_function_t task_function; // Pointer to the task function.
  uint32_t period_us;            // Task execution period in us.
  uint32_t next_execution_us;    // Next execution due in us.
} task_t;

/** Public functions. *********************************************************/

/**
 * @brief Initialize scheduler.
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
