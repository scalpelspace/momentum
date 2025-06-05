/*******************************************************************************
 * @file can.h
 * @brief General STM32L432KC driver abstracting STM32 HAL: CAN.
 *******************************************************************************
 */

#ifndef PWM_NODE__CAN_H
#define PWM_NODE__CAN_H

/** Includes. *****************************************************************/

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_can.h"

/** Definitions. **************************************************************/

#define MAX_SIGNALS_PER_MESSAGE 8 // Maximum signals allowed per message.

/** STM32 port and pin configs. ***********************************************/

extern CAN_HandleTypeDef hcan1;

/** Public types. *************************************************************/

/**
 * @brief Define function pointer types for processing CAN bus messages.
 *
 * Two distinct handlers are defined:
 *   1. can_rx_handler_t: For decoding received CAN messages.
 *   2. can_tx_handler_t: For encoding data into CAN messages for transmission.
 */
typedef void (*can_rx_handler_t)(CAN_RxHeaderTypeDef *header, uint8_t *data);
typedef void (*can_tx_handler_t)(uint8_t *data_out);

/**
 * @brief Enumeration for CAN signal byte order.
 */
typedef enum {
  CAN_LITTLE_ENDIAN = 0, // Little Endian byte order.
  CAN_BIG_ENDIAN = 1     // Big Endian byte order.
} can_byte_order_t;

/**
 * @brief Struct defining a CAN message signal.
 *
 * This struct describes an individual signal within a CAN message.
 * It includes fields for bit-position, length, scaling, and validation.
 * An optional name field aids in debugging and logging.
 */
typedef struct {
  const char *name;            // Optional signal identifier (for debugging).
  uint8_t start_bit;           // Start bit-position (0-63 for 8-byte CAN).
  uint8_t bit_length;          // Length of the signal in bits.
  can_byte_order_t byte_order; // Byte order: little or big endian.
  float scale;     // Scaling factor to convert raw value to physical value.
  float offset;    // Offset to apply after scaling.
  float min_value; // Minimum physical value (optional validation).
  float max_value; // Maximum physical value (optional validation).
} can_signal_t;

/**
 * @brief Struct defining a CAN message configuration.
 *
 * This struct holds the static configuration for a CAN message, including its
 * ID, optional name, data length, and the associated handler functions. Signals
 * for the message are statically allocated in a fixed-size array.
 */
typedef struct {
  const char *name;    // Optional message name (for debugging).
  uint32_t message_id; // CAN message ID.
  uint32_t id_mask; // ID mask for filtering (supports ranges or specific IDs).
  uint8_t dlc;      // Data Length Code.
  can_rx_handler_t rx_handler; // Function pointer for receiving (decoding).
  can_tx_handler_t tx_handler; // Function pointer for transmitting (encoding).
  can_signal_t signals[MAX_SIGNALS_PER_MESSAGE]; // Statically allocation.
  uint8_t signal_count; // Number of valid signals in the array.
} can_message_t;

/** Public functions. *********************************************************/

/**
 * @brief Normalize physical values (uint32_t) to uint32_t for raw CAN signals.
 *
 * @param physical_value Physical uint32_t value.
 * @param signal Reference signal to ensure clamping and normalization.
 *
 * @return Normalized raw uint32_t data.
 */
uint32_t uint_to_raw(uint32_t physical_value, const can_signal_t *signal);

/**
 * @breif Normalize physical values (float) to uint32_t for raw CAN signals.
 *
 * @param physical_value Physical float value.
 * @param signal Reference signal to ensure clamping and normalization.
 *
 * @return Normalized raw uint32_t data.
 */
uint32_t float_to_raw(float physical_value, const can_signal_t *signal);

/**
 * @breif Normalize physical values (double) to uint32_t for raw CAN signals.
 *
 * @param physical_value Physical double value.
 * @param signal Reference signal to ensure clamping and normalization.
 *
 * @return Normalized raw uint32_t data.
 */
uint32_t double_to_raw(double physical_value, const can_signal_t *signal);

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
