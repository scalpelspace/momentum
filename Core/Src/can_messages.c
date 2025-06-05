/** Auto-generated CAN message definitions from DBC file. */

#include "can_messages.h"

const can_message_t dbc_messages[] = {
    {
        .name = "example_message",
        .message_id = 100,
        .id_mask = 0xFFFFFFFF,
        .dlc = 1,
        .rx_handler = 0,
        .tx_handler = 0,
        .signal_count = 1,
        .signals =
            {
                {
                    .name = "example_signal",
                    .start_bit = 0,
                    .bit_length = 8,
                    .byte_order = CAN_LITTLE_ENDIAN,
                    .scale = 1.0f,
                    .offset = 0.0f,
                    .min_value = 0.0f,
                    .max_value = 255.0f,
                },
            },
    },
};

const int dbc_message_count = sizeof(dbc_messages) / sizeof(dbc_messages[0]);
