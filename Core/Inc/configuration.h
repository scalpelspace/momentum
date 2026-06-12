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
#define MOMENTUM_VERSION_MAJOR (0u)     // uint8_t number.
#define MOMENTUM_VERSION_MINOR (4u)     // uint8_t number.
#define MOMENTUM_VERSION_PATCH (9u)     // uint8_t number.
#define MOMENTUM_VERSION_IDENTIFIER 'p' // ASCII character.

// CAN ID standard.
#define DEFAULT_CAN_NODE_ID (0u)       // Unassigned.
#define ALLOW_CAN_NODE_ID_REASSIGNMENT // Allow node ID reassignment always.

// Full telemetry flood on CAN bus intended for debug/development purposes.
#define MOMENTUM_FULL_CAN_TELEMETRY

// Full telemetry flood on UART intended for debug/development purposes.
// #define MOMENTUM_FULL_COMM_TELEMETRY

/** BNO085 IMU sensor configurations. *****************************************/

// SH2_GAME_ROTATION_VECTOR report interval (in us), use 0 for disabled.
#define MOMENTUM_QUATERNION_REPORT_US (10000u)
// Select 6-DOF or 9-DOF quaternion vector (0 for 6-DOF, 1 for 9-DOF).
#define MOMENTUM_USE_9DOF_QUATERNION 0

// SH2_GYROSCOPE_CALIBRATED report interval (in us), use 0 for disabled.
#define MOMENTUM_GYROSCOPE_REPORT_US (10000u)

// SH2_MAGNETIC_FIELD_CALIBRATED report interval (in us), use 0 for disabled.
#define MOMENTUM_MAGNETOMETER_REPORT_US (40000u)

// SH2_ACCELEROMETER report interval (in us), use 0 for disabled.
#define MOMENTUM_ACCELEROMETER_REPORT_US (10000u)

// SH2_LINEAR_ACCELERATION report interval (in us), use 0 for disabled.
#define MOMENTUM_LINEAR_ACCELEROMETER_REPORT_US (0u)

// SH2_GRAVITY report interval (in us), use 0 for disabled.
#define MOMENTUM_GRAVITY_ACCELEROMETER_REPORT_US (0u)

#endif
