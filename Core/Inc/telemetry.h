/*******************************************************************************
 * @file telemetry.h
 * @brief General CAN message transmission functions.
 *******************************************************************************
 */

#ifndef MOMENTUM__TELEMETRY_H
#define MOMENTUM__TELEMETRY_H

/** Public functions. *********************************************************/

void can_tx_state(void);
void can_tx_barometric(void);
void can_tx_gnss1(void);
void can_tx_gnss2(void);
void can_tx_gnss3(void);
void can_tx_imu1(void);
void can_tx_imu2(void);
void can_tx_imu3(void);
void can_tx_imu4(void);
void can_tx_imu5(void);
void comm_tx_state(void);
void comm_tx_barometric(void);
void comm_tx_gnss1(void);
void comm_tx_gnss2(void);
void comm_tx_gnss3(void);
void comm_tx_imu1(void);
void comm_tx_imu2(void);
void comm_tx_imu3(void);
void comm_tx_imu4(void);
void comm_tx_imu5(void);
void comm_tx_rtc(void);

#endif
