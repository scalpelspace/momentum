/*******************************************************************************
 * @file can.h
 * @brief General STM32L432KC driver abstracting STM32 HAL: CAN.
 *******************************************************************************
 */

#ifndef MOMENTUM__CAN_H
#define MOMENTUM__CAN_H

/** Includes. *****************************************************************/

#include "can_driver/can_id_allocatee.h"
#include "momentum_driver.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_can.h"

/** STM32 port and pin configs. ***********************************************/

extern CAN_HandleTypeDef hcan1;

// CAN.
#define HCAN hcan1

/** Public variables. *********************************************************/

// Mutable runtime copy.
extern can_message_t mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_COUNT];

extern can_node_id_t can_node_id;

/** Public functions. *********************************************************/

/**
 * @brief Initialize CAN.
 */
void can_init(void);

/**
 * @brief Initialize CAN DBC.
 */
void can_db_init(void);

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

/**
 * @brief General CAN transmit function for CAN ID implementation.
 *
 * @param msg
 * @param data
 *
 * @return CAN bus message transmit status.
 * @retval true -> HAL_OK response.
 * @retval false -> non HAL_OK response.
 */
bool can_tx_direct(const can_message_t *msg, const uint8_t data[8]);

/**
 * @brief General node ID completion callback for CAN ID implementation.
 *
 * @param node_id
 */
void allocatee_complete(can_node_id_t node_id);

#endif
