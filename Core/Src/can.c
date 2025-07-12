/*******************************************************************************
 * @file can.c
 * @brief General STM32L432KC driver abstracting STM32 HAL: CAN.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "can.h"
#include "can_dbc.h"
#include "math.h"

/** Private variables. ********************************************************/

// Transmit variables.
CAN_TxHeaderTypeDef tx_header;
uint8_t tx_buffer[8];
uint32_t tx_mailbox;

/** Private functions. ********************************************************/

/**
 * @brief Process CAN bus messages based on the configured message structs.
 *
 * This function iterates over the statically allocated CAN bus messages and
 * checks whether the incoming message's ID and DLC match the configuration.
 * If a match is found and a receiver handler exists, that handler is invoked.
 *
 * @param header Pointer to the CAN RX header.
 * @param data Pointer to the raw data of the CAN message.
 */
void process_can_message(CAN_RxHeaderTypeDef *header, uint8_t *data) {
  for (int i = 0; i < dbc_message_count; i++) {
    // Check if the message ID matches.
    if ((header->StdId & dbc_messages[i].id_mask) ==
        dbc_messages[i].message_id) {

      // Check if the message DLC matches, or if no check required (dlc == 0).
      if (dbc_messages[i].dlc == 0 || header->DLC == dbc_messages[i].dlc) {

        // Call the rx_handler if it exists.
        if (dbc_messages[i].rx_handler) {
          dbc_messages[i].rx_handler(header, data);
        }

      } else {
        // Handle ID/DLC mismatch fault.
        // can_fault(); // TODO: IMPLEMENT.
        return;
      }
    }
  }
}

/**
 * @brief Extract a signal value from a CAN message payload.
 *
 * This function extracts the raw signal value from the provided data array
 * using the specified bit-position, length, and byte order. It then applies
 * scaling and offset to convert the raw value into a physical value.
 *
 * @param signal Pointer to the CAN signal configuration.
 * @param data Pointer to the raw CAN message payload.
 *
 * @return The decoded physical signal value.
 */
float decode_signal(const can_signal_t *signal, const uint8_t *data) {
  uint64_t raw_value = 0;

  // Extract raw bits from the CAN message payload.
  for (int i = 0; i < signal->bit_length; i++) {
    int bit_position = signal->start_bit + i;
    int byte_index = bit_position / 8;
    int bit_index = bit_position % 8;

    // Use the enumerated constant for byte order.
    if (signal->byte_order == CAN_BIG_ENDIAN) {
      raw_value |= ((data[byte_index] >> (7 - bit_index)) & 0x1)
                   << (signal->bit_length - 1 - i);
    } else { // CAN_LITTLE_ENDIAN.
      raw_value |= ((data[byte_index] >> bit_index) & 0x1) << i;
    }
  }

  // Convert raw value to physical value by applying scale and offset.
  return ((float)raw_value * signal->scale) + signal->offset;
}

/**
 * @brief Packs a uint32_t value into the CAN message data buffer.
 *
 * @param signal Pointer to the signal definition.
 * @param data Pointer to the CAN data array.
 * @param physical_value The physical value to encode.
 */
static void pack_signal_raw32(const can_signal_t *signal, uint8_t *data,
                              uint32_t raw_value) {
  // Mask off any bits above bit_length.
  if (signal->bit_length < 32) {
    raw_value &= ((1UL << signal->bit_length) - 1UL);
  }

  // Pack each bit into data[].
  for (uint32_t bit = 0; bit < signal->bit_length; ++bit) {
    // Determine the absolute bit position in the 64‑bit CAN payload.
    uint32_t bit_pos = (signal->byte_order == CAN_LITTLE_ENDIAN)
                           ? (signal->start_bit + bit)  // Intel/little‑endian.
                           : (signal->start_bit - bit); // Motorola/big‑endian.

    uint32_t byte_index = bit_pos / 8;
    uint32_t bit_index = bit_pos % 8;

    // Extract the next raw bit, then OR it into the right place.
    uint8_t raw_bit = (raw_value >> bit) & 0x1U;
    data[byte_index] |= (raw_bit << bit_index);
  }
}

/** User implementations of STM32 CAN NVIC HAL (overwriting HAL). *************/

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  // Receive the CAN message.
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
    // Process the received message.
    process_can_message(&rx_header, rx_data);
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  // Receive the CAN message.
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data) == HAL_OK) {
    // Process the received message.
    process_can_message(&rx_header, rx_data);
  }
}

/** Public functions. *********************************************************/

uint32_t uint_to_raw(uint32_t physical_value, const can_signal_t *signal) {
  // Clamp physical value into [min, max].
  if (physical_value < (uint32_t)signal->min_value)
    physical_value = (uint32_t)signal->min_value;
  if (physical_value > (uint32_t)signal->max_value)
    physical_value = (uint32_t)signal->max_value;

  // Normalize into raw units (cast to float just for the calculation).
  float normalized = ((float)physical_value - signal->offset) / signal->scale;

  // Round to nearest uint32_t.
  uint32_t raw = (uint32_t)roundf(normalized);

  return raw;
}

uint32_t float_to_raw(float physical_value, const can_signal_t *signal) {
  // Clamp physical value into [min, max].
  if (physical_value < signal->min_value)
    physical_value = signal->min_value;
  if (physical_value > signal->max_value)
    physical_value = signal->max_value;

  // Normalize into raw units.
  float normalized = (physical_value - signal->offset) / signal->scale;

  // Round to nearest uint32_t.
  uint32_t raw = (uint32_t)roundf(normalized);

  return raw;
}

uint32_t double_to_raw(double physical_value, const can_signal_t *signal) {
  // Clamp physical value into [min, max].
  if (physical_value < signal->min_value)
    physical_value = signal->min_value;
  if (physical_value > signal->max_value)
    physical_value = signal->max_value;

  // Normalize into raw units.
  double normalized = (physical_value - signal->offset) / signal->scale;

  // Round to nearest uint32_t.
  uint32_t raw = (uint32_t)llround(normalized);

  return raw;
}

void can_init(void) {
  // Configure CAN bus filters.
  CAN_FilterTypeDef can_filter_config;

  can_filter_config.FilterActivation = CAN_FILTER_ENABLE; // Enable the filter.
  can_filter_config.FilterBank = 0; // Assign filter to bank 0.
  can_filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0; // Use FIFO0.
  // Set filter to accept all IDs.
  can_filter_config.FilterIdHigh = 0x0000;
  can_filter_config.FilterIdLow = 0x0000;
  // Set mask to 0 so that no bits are filtered, allow all messages.
  can_filter_config.FilterMaskIdHigh = 0x0000;
  can_filter_config.FilterMaskIdLow = 0x0000;
  // Use mask mode (compare message ID with mask).
  can_filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
  // Use 32-bit filter scale.
  can_filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
  // Filter bank config for single CAN setup.
  can_filter_config.SlaveStartFilterBank = 14;

  // Apply filter settings to CAN1.
  HAL_CAN_ConfigFilter(&hcan1, &can_filter_config);

  // Start CAN1.
  HAL_CAN_Start(&hcan1);

  // Enable interrupts.
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  // Setup CAN general members on the transmit header.
  tx_header.IDE = CAN_ID_STD;
  // Remaining members (DLC and StdId) are configured per message on transmit.
}

HAL_StatusTypeDef can_send_message_raw32(CAN_HandleTypeDef *h_can_x,
                                         const can_message_t *msg,
                                         const uint32_t signal_values[]) {
  uint8_t data[8] = {0};

  for (int i = 0; i < msg->signal_count; ++i) {
    pack_signal_raw32(&msg->signals[i], data, signal_values[i]);
  }

  // Prepare the CAN transmit header.
  tx_header.StdId = msg->message_id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = msg->dlc;

  return HAL_CAN_AddTxMessage(h_can_x, &tx_header, data, &tx_mailbox);
}
