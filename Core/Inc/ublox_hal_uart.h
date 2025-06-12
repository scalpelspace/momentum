/*******************************************************************************
 * @file ublox_hal_uart.h
 * @brief u-blox NMEA protocol functions: abstracting STM32 HAL: UART.
 *******************************************************************************
 */

#ifndef MOMENTUM__UBLOX_HAL_UART_H
#define MOMENTUM__UBLOX_HAL_UART_H

/** Includes. *****************************************************************/

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include <stdio.h>
#include <string.h>

/** STM32 port and pin configs. ***********************************************/

extern UART_HandleTypeDef huart2;

// UART.
#define UBLOX_HUART huart2
#define UBLOX_INIT_BAUD_RATE 115200 // Reconfigure during init via software.

// GPIO output for reset.
#define UBLOX_RESETN_PORT GPIOC
#define UBLOX_RESETN_PIN GPIO_PIN_15

/** Definitions. **************************************************************/

#define UBLOX_RX_BUFFER_SIZE 128 // Based on NEMA specifications.
// Based on $GNGGA maximum 82 characters/bytes with some overhead.

/** Public types. *************************************************************/

/**
 * @brief A compact struct containing exactly the three NMEA‐fields used for
 *        deciding which of the nine position‐fix types is most accurate.
 *
 * Once these three values are determined, a higher precision
 * nmea_position_fix_t classification can be made (see classify_position_fix).
 */
typedef struct {
  char status;     // RMC/GLL "status" field: ( 'A' or 'V' ).
  uint8_t quality; // GGA "quality" field: ( 0..6 ).
  char pos_mode;   // RMC/GNS "pos_mode" field: ( 'N','A','D','E','R','F' ).
} nmea_fix_flags_t;

/**
 * @brief ENUMs to describe the (NMEA 4.10+) possible position fix flags.
 */
typedef enum {
  FIX_TYPE_UNDETERMINED = 0,         // Classification pending.
  FIX_TYPE_NO_FIX = 1,               // No fix.
  FIX_TYPE_GNSS_LIMITS_EXCEEDED = 2, // Regulation limited.
  FIX_TYPE_DR_LIMITS_EXCEEDED = 3,   // DR = Dead Reckoning. Regulation limited.
  FIX_TYPE_DR_FIX = 4,               // DR = Dead Reckoning.
  FIX_TYPE_RTK_FLOAT = 5,            // RTK = Real Time Kinetic.
  FIX_TYPE_RTK_FIX = 6,              // RTK = Real Time Kinetic.
  FIX_TYPE_GNSS_FIX = 7              // GNSS 2D, GNSS 3D or GNSS + DR combined.
} nmea_position_fix_t;

/**
 * @brief Struct to store GPS data.
 */
typedef struct {
  nmea_position_fix_t position_fix;
  nmea_fix_flags_t position_flags;
  uint8_t year;       // RTC date, year from GPS satellite.
  uint8_t month;      // RTC date, month from GPS satellite.
  uint8_t day;        // RTC date, day from GPS satellite.
  uint8_t hour;       // RTC time, hour from GPS satellite.
  uint8_t minute;     // RTC time, minute from GPS satellite.
  uint8_t second;     // RTC time, second from GPS satellite.
  float latitude;     // Latitude in decimal degrees.
  char lat_dir;       // Latitude direction (N/S).
  float longitude;    // Longitude in decimal degrees.
  char lon_dir;       // Longitude direction (E/W).
  float altitude_m;   // Altitude in meters.
  float geoid_sep_m;  // Geoid separation.
  float speed_knots;  // Speed over the ground in knots.
  float course_deg;   // Course over ground in degrees.
  float magnetic_deg; // Magnetic variation in degrees.
  char mag_dir;       // Magnetic variation direction (E/W).
  uint8_t satellites; // Number of Satellites.
  float hdop;         // Horizontal Dilution of Precision (HDOP).
} ublox_data_t;

/** Public variables. *********************************************************/

extern ublox_data_t gps_data;

/** User implementations of STM32 NVIC HAL (overwriting HAL). *****************/

void HAL_UART_RxCpltCallback_ublox(UART_HandleTypeDef *huart);
void USART2_IRQHandler_ublox(UART_HandleTypeDef *huart);

/** Public functions. *********************************************************/

/**
 * @brief Initialize the u-blox module.
 */
void ublox_init(void);

/**
 * @brief Reset the u-blox module.
 *
 * @note Only for critical situations for recovery, triggers cold start.
 */
void ublox_reset(void);

/**
 * @brief Build and send a PUBX,41 message to reconfigure the UART Baud-rate.
 *
 * @param baud_rate Desired UART baud-rate (e.g. 9600, 115200).
 */
void ublox_set_baud_rate(uint32_t baud_rate);

/**
 * @brief Change the dynamic model in RAM (CFG-NAV5 -> dynModel).
 *
 * Sends a UBX-CFG-NAV5 packet (class=0x06, id=0x24, length=36). Only the
 * dynModel field is modified. fixMode is hard-coded to 0x03 (Auto 2D/3D).
 *
 * @param dyn_model One of {0,2,3,4,5,6,7,8,9,10,11,12} corresponding to:
 *                 0=Portable, 2=Stationary, 3=Pedestrian, 4=Automotive,
 *                 5=Sea, 6=Airborne<1g, 7=Airborne<2g, 8=Airborne<4g,
 *                 9=Wrist, 10=Bike, 11=Mower, 12=E-scooter.
 */
void ublox_set_dynamic_model(uint8_t dyn_model);

/**
 * @brief Disable every NMEA sentence on UART except GGA (0x00) and RMC (0x04).
 *
 * This function sends one UBX-CFG-MSG packet (class=0x06, id=0x01, length=8)
 * for each message ID in the "disable_list[]" array. Each packet has:
 *
 *  payload[0] = 0xF0      (msgClass = NMEA)
 *  payload[1] = <msgID>   (e.g. 0x01 for GLL, 0x02 for GSA, etc.)
 *  payload[2] = 0x00      (rateUART = 0 -> disable on UART)
 *  payload[3] = 0x00      (rateI2C = 0, not used here)
 *  payload[4] = 0x00      (rateUSB = 0)
 *  payload[5] = 0x00      (rateSPI = 0)
 *  payload[6..7] = 0x0000 (reserved)
 *
 * The 14-byte packet structure is:
 *
 *  [0]  = 0xB5
 *  [1]  = 0x62
 *  [2]  = 0x06    (CFG class)
 *  [3]  = 0x01    (MSG ID)
 *  [4]  = 0x08    (payload length LSB = 8)
 *  [5]  = 0x00    (payload length MSB = 0)
 *  [6]  = 0xF0    (msgClass = NMEA)
 *  [7]  = <msgID> (e.g. 0x01 = GLL, 0x02 = GSA, etc.)
 *  [8]  = 0x00    (rateUART = 0)
 *  [9]  = 0x00    (rateI2C = 0)
 *  [10] = 0x00    (rateUSB = 0)
 *  [11] = 0x00    (rateSPI = 0)
 *  [12] = 0x00    (reserved LSB)
 *  [13] = 0x00    (reserved MSB)
 *  [14] = CK_A    (computed checksum over bytes [2..13])
 *  [15] = CK_B
 *
 * After transmission, the module will no longer output any of the listed NMEA
 * sentences on UART1 (only GGA and RMC will remain enabled, assuming they are
 * left at a nonzero rate).
 */
void ublox_disable_other_nmea_messages(void);

/**
 * @brief Configure SAM-M10Q for 10 Hz navigation and NMEA output (GGA/RMC).
 *
 * This function:
 * 1) Sends CFG-RATE to set measurement period = 100 ms (10 Hz).
 * 2) Disables all other NMEA sentences (GLL, GSA, GSV, VTG, etc.).
 * 3) Enables GGA @ 10 Hz and RMC @ 10 Hz on UART1.
 *
 * Must be called after the UART has been initialized at the desired baud.
 */
void ublox_enable_10hz(void);

#endif
