/*******************************************************************************
 * @file can.h
 * @brief General STM32L432KC driver abstracting STM32 HAL: CAN.
 *******************************************************************************
 */

#ifndef MOMENTUM__CAN_H
#define MOMENTUM__CAN_H

/** Includes. *****************************************************************/

#include "momentum_can_driver.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_can.h"

/** STM32 port and pin configs. ***********************************************/

extern CAN_HandleTypeDef hcan1;

/** Public functions. *********************************************************/

/**
 * @brief Initialize CAN.
 */
void can_init(void);

/**
 * @brief Send uint32_t data CAN message on h_can_x with can_message_t
 * reference.
 *
 * This function takes a pointer to a can_message_t definition and an array of
 * uint32_t type elements (one per signal in the message). These values are the
 * converted physical values based on the DBC.
 *
 * @param h_can_x STM32 CAN_HandleTypeDef type to decide which CAN bus to use.
 * @param msg Pointer to the static CAN message definition.
 * @param signal_values Array of physical values for each signal in the message.
 *                      The array length must equal msg->signal_count.
 *
 * @return HAL_StatusTypeDef HAL status indicating whether the transmission
 *         was successful.
 *
 * @example
 * ```
 * float signal_value = 1.0f; // Example physical value for the "state".
 * HAL_StatusTypeDef status =
 *     can_send_message_generic(&dbc_messages[0], &signal_value);
 *
 * if (status != HAL_OK) {
 *   // Handle transmission error.
 *   can_fault();
 * }
 * ```
 */
HAL_StatusTypeDef can_send_message_raw32(CAN_HandleTypeDef *h_can_x,
                                         const can_message_t *msg,
                                         const uint32_t signal_values[]);

#endif
