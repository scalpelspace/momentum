/*******************************************************************************
 * @file telemetry.h
 * @brief General telemetry functions including CAN.
 *******************************************************************************
 */

#ifndef MOMENTUM__TELEMETRY_H
#define MOMENTUM__TELEMETRY_H

/** Public functions. *********************************************************/

void can_tx_state(void);
void can_tx_barometric(void);
void can_tx_gps1(void);
void can_tx_gps2(void);
void can_tx_gps3(void);
void can_tx_imu1(void);
void can_tx_imu2(void);
void can_tx_imu3(void);
void can_tx_imu4(void);
void can_tx_imu5(void);
void can_tx_rtc(void);

#endif
