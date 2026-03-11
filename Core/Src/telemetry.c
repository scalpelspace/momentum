/*******************************************************************************
 * @file telemetry.c
 * @brief General CAN message transmission functions.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "telemetry.h"
#include "bmp390_runner.h"
#include "bno085_runner.h"
#include "can.h"
#include "momentum_can_dbc.h"
#include "rtc.h"
#include "ublox_hal_uart.h"

/** Public functions. *********************************************************/

void can_tx_state(void) {
  const can_message_t state_msg = mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_STATE];
  const uint32_t state_sigs[1] = {0}; // TODO: Hardcoded state.
  can_send_message_raw32(&hcan1, &state_msg, state_sigs);
}

void can_tx_barometric(void) {
  const can_message_t pressure_msg =
      mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_BAROMETRIC];
  uint32_t pressure_sigs[3] = {0};
  const float pressure_source_sigs[3] = {bmp390_pressure, bmp390_temperature,
                                         (float)0}; // TODO: Hardcoded state.
  for (int i = 0; i < pressure_msg.signal_count; ++i) {
    pressure_sigs[i] =
        physical_to_raw(pressure_source_sigs[i], &pressure_msg.signals[i]);
  }
  can_send_message_raw32(&hcan1, &pressure_msg, pressure_sigs);
}

void can_tx_gnss1(void) {
  const can_message_t gnss1_msg = mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_GNSS1];
  uint32_t gnss1_sigs[2] = {0};
  const float gnss1_source_sigs[2] = {gnss_data.latitude, gnss_data.longitude};
  for (int i = 0; i < gnss1_msg.signal_count; ++i) {
    gnss1_sigs[i] =
        physical_to_raw(gnss1_source_sigs[i], &gnss1_msg.signals[i]);
  }
  can_send_message_raw32(&hcan1, &gnss1_msg, gnss1_sigs);
}

void can_tx_gnss2(void) {
  const can_message_t gnss2_msg = mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_GNSS2];
  uint32_t gnss2_sigs[5] = {0};
  const float gnss2_source_sigs[5] = {
      gnss_data.speed_knots, gnss_data.course_deg, gnss_data.position_fix,
      gnss_data.satellites, gnss_data.hdop};
  for (int i = 0; i < gnss2_msg.signal_count; ++i) {
    gnss2_sigs[i] =
        physical_to_raw(gnss2_source_sigs[i], &gnss2_msg.signals[i]);
  }
  can_send_message_raw32(&hcan1, &gnss2_msg, gnss2_sigs);
}

void can_tx_gnss3(void) {
  const can_message_t gnss3_msg = mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_GNSS3];
  uint32_t gnss3_sigs[3] = {0};
  const float gnss3_source_sigs[3] = {gnss_data.altitude_m,
                                      gnss_data.geoid_sep_m,
                                      (float)0}; // TODO: Hardcoded state.
  for (int i = 0; i < gnss3_msg.signal_count; ++i) {
    gnss3_sigs[i] =
        physical_to_raw(gnss3_source_sigs[i], &gnss3_msg.signals[i]);
  }
  can_send_message_raw32(&hcan1, &gnss3_msg, gnss3_sigs);
}

void can_tx_imu1(void) {
  const can_message_t imu1_msg = mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_IMU1];
  uint32_t imu1_sigs[4] = {0};
  const float imu1_source_sigs[4] = {bno085_quaternion_i, bno085_quaternion_j,
                                     bno085_quaternion_k,
                                     bno085_quaternion_real};
  for (int i = 0; i < imu1_msg.signal_count; ++i) {
    imu1_sigs[i] = physical_to_raw(imu1_source_sigs[i], &imu1_msg.signals[i]);
  }
  can_send_message_raw32(&hcan1, &imu1_msg, imu1_sigs);
}

void can_tx_imu2(void) {
  const can_message_t imu2_msg = mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_IMU2];
  uint32_t imu2_sigs[3] = {0};
  const float imu2_source_sigs[3] = {bno085_gyro_x, bno085_gyro_y,
                                     bno085_gyro_z};
  for (int i = 0; i < imu2_msg.signal_count; ++i) {
    imu2_sigs[i] = physical_to_raw(imu2_source_sigs[i], &imu2_msg.signals[i]);
  }
  can_send_message_raw32(&hcan1, &imu2_msg, imu2_sigs);
}

void can_tx_imu3(void) {
  const can_message_t imu3_msg = mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_IMU3];
  uint32_t imu3_sigs[3] = {0};
  const float imu3_source_sigs[3] = {bno085_accel_x, bno085_accel_y,
                                     bno085_accel_z};
  for (int i = 0; i < imu3_msg.signal_count; ++i) {
    imu3_sigs[i] = physical_to_raw(imu3_source_sigs[i], &imu3_msg.signals[i]);
  }
  can_send_message_raw32(&hcan1, &imu3_msg, imu3_sigs);
}

void can_tx_imu4(void) {
  const can_message_t imu4_msg = mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_IMU4];
  uint32_t imu4_sigs[3] = {0};
  const float imu4_source_sigs[3] = {bno085_lin_accel_x, bno085_lin_accel_y,
                                     bno085_lin_accel_z};
  for (int i = 0; i < imu4_msg.signal_count; ++i) {
    imu4_sigs[i] = physical_to_raw(imu4_source_sigs[i], &imu4_msg.signals[i]);
  }
  can_send_message_raw32(&hcan1, &imu4_msg, imu4_sigs);
}

void can_tx_imu5(void) {
  const can_message_t imu5_msg = mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_IMU5];
  uint32_t imu5_sigs[3] = {0};
  const float imu5_source_sigs[3] = {bno085_gravity_x, bno085_gravity_y,
                                     bno085_gravity_z};
  for (int i = 0; i < imu5_msg.signal_count; ++i) {
    imu5_sigs[i] = physical_to_raw(imu5_source_sigs[i], &imu5_msg.signals[i]);
  }
  can_send_message_raw32(&hcan1, &imu5_msg, imu5_sigs);
}

void comm_tx_state(void) {
  printf("s=%u\r\n", 0); // TODO: Hardcoded state.
}

void comm_tx_barometric(void) {
  printf("pres=%.3f,temp=%.3f,s=%u\r\n", bmp390_pressure, bmp390_temperature,
         0); // TODO: Hardcoded state.
}

void comm_tx_gnss1(void) {
  printf("lat=%.3f,lon=%.3f\r\n", gnss_data.latitude, gnss_data.longitude);
}

void comm_tx_gnss2(void) {
  printf("sp=%.3f,cdeg=%.3f,pf=%u,sat=%u,hdop=%.3f\r\n", gnss_data.speed_knots,
         gnss_data.course_deg, gnss_data.position_fix, gnss_data.satellites,
         gnss_data.hdop);
}

void comm_tx_gnss3(void) {
  printf("alt=%.3f,gid=%.3f,s=%u\r\n", gnss_data.altitude_m,
         gnss_data.geoid_sep_m, 0); // TODO: Hardcoded state.
}

void comm_tx_imu1(void) {
  printf("qi=%.3f,qj=%.3f,qk=%.3f,qr=%.3f\r\n", bno085_quaternion_i,
         bno085_quaternion_j, bno085_quaternion_k, bno085_quaternion_real);
}

void comm_tx_imu2(void) {
  printf("gx=%.3f,gy=%.3f,gz=%.3f\r\n", bno085_gyro_x, bno085_gyro_y,
         bno085_gyro_z);
}

void comm_tx_imu3(void) {
  printf("ax=%.3f,ay=%.3f,az=%.3f\r\n", bno085_accel_x, bno085_accel_y,
         bno085_accel_z);
}

void comm_tx_imu4(void) {
  printf("lax=%.3f,lay=%.3f,laz=%.3f\r\n", bno085_lin_accel_x,
         bno085_lin_accel_y, bno085_lin_accel_z);
}

void comm_tx_imu5(void) {
  printf("gvx=%.3f,gvy=%.3f,gvz=%.3f\r\n", bno085_gravity_x, bno085_gravity_y,
         bno085_gravity_z);
}

void comm_tx_rtc(void) {
  // Get the date and time.
  RTC_DateTypeDef date;
  RTC_TimeTypeDef time;
  HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
  // TODO: Hardcoded state.
  printf("s=%u,d=%u-%u-%u,wd=%u,t=%u:%u:%u\r\n", 0, 2000 + date.Year,
         date.Month, date.Date, date.WeekDay, time.Hours, time.Minutes,
         time.Seconds);
}
