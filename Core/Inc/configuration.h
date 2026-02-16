/*******************************************************************************
 * @file configuration.h <- TODO: DEV CONFIGURATIONS!
 * @brief High level hardcoded configuration related declarations/definitions.
 *******************************************************************************
 */

#ifndef MOMENTUM__CONFIGURATION_H
#define MOMENTUM__CONFIGURATION_H

/** Configuration definitions. ************************************************/

// Centralized short name.
#define SCALPELSPACE_SHORT_NAME "momentum" // Short name.

// Software version: `MAJOR.MINOR.PATCH-IDENTIFIER`.
#define MOMENTUM_VERSION_MAJOR (0U)     // uint8_t number.
#define MOMENTUM_VERSION_MINOR (2U)     // uint8_t number.
#define MOMENTUM_VERSION_PATCH (4U)     // uint8_t number.
#define MOMENTUM_VERSION_IDENTIFIER 'p' // ASCII character.

// Full telemetry flood on CAN bus intended for debug/development purposes.
#define MOMENTUM_FULL_CAN_TELEMETRY

// Full telemetry flood on UART intended for debug/development purposes.
// #define MOMENTUM_FULL_COMM_TELEMETRY

#endif
