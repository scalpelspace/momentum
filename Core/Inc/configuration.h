/*******************************************************************************
 * @file configuration.h <- TODO: DEV CONFIGURATIONS!
 * @brief High level hardcoded configuration related declarations/definitions.
 *******************************************************************************
 */

#ifndef MOMENTUM__CONFIGURATION_H
#define MOMENTUM__CONFIGURATION_H

/** Configuration definitions. ************************************************/

// Enables use of SPI3 and PA15 (SPI3 CS) for W25Qxx operations.
// WARNING: Overwrites HAL configs!
//#define MOMENTUM_W25QXX_ENABLE
//#define MOMENTUM_W25QXX_EXPECTED_MANUF 0xEF         // Example for W25Q128.
//#define MOMENTUM_W25QXX_EXPECTED_MEM_TYPE 0x40      // Example for W25Q128.
//#define MOMENTUM_W25QXX_EXPECTED_CAPACITY 0x18      // Example for W25Q128.
//#define MOMENTUM_W25QXX_LOGGER_ADDR_START 0x000000U // Example for W25Q128.
//#define MOMENTUM_W25QXX_LOGGER_ADDR_END 0x1FFFFFU   // Example for W25Q128.

// Enables low level USART1 comm functions for USB serial interface operations.
//#define MOMENTUM_COMM_ENABLE

// Full telemetry flood on CAN bus intended for debug/development purposes.
//#define FULL_CAN_TELEMETRY

#endif
