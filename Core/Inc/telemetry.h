/*******************************************************************************
 * @file telemetry.h
 * @brief General CAN message transmission functions.
 *******************************************************************************
 */

#ifndef MOMENTUM__TELEMETRY_H
#define MOMENTUM__TELEMETRY_H

/** Public functions. *********************************************************/

void can_tx_barometric(void);
void can_tx_gnss1(void);
void can_tx_gnss2(void);
void can_tx_gnss3(void);
void can_tx_quaternion(void);
void can_tx_mag(void);
void can_tx_gyro(void);
void can_tx_accel(void);
void can_tx_lin_accel(void);
void can_tx_gravity(void);

void comm_tx_barometric(void);
void comm_tx_gnss1(void);
void comm_tx_gnss2(void);
void comm_tx_gnss3(void);
void comm_tx_quaternion(void);
void comm_tx_mag(void);
void comm_tx_gyro(void);
void comm_tx_accel(void);
void comm_tx_lin_accel(void);
void comm_tx_gravity(void);
void comm_tx_rtc(void);

#endif
