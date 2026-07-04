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

void can_tx_barometric(void) {
  const can_message_t *msg = &mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_BAROMETRIC];
  const uint32_t values[] = {
      physical_to_raw(bmp390_pressure, &msg->signals[0]),
      physical_to_raw(bmp390_temperature, &msg->signals[1]),
      physical_to_raw((float)0, &msg->signals[2]), // TODO: Hardcoded state.
  };
  can_send_message_raw32(&hcan1, msg, values);
}

void can_tx_gnss1(void) {
  const can_message_t *msg = &mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_GNSS1];
  const uint32_t values[] = {
      physical_to_raw(gnss_data.latitude, &msg->signals[0]),
      physical_to_raw(gnss_data.longitude, &msg->signals[1]),
  };
  can_send_message_raw32(&hcan1, msg, values);
}

void can_tx_gnss2(void) {
  const can_message_t *msg = &mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_GNSS2];
  const uint32_t values[] = {
      physical_to_raw(gnss_data.speed_knots, &msg->signals[0]),
      physical_to_raw(gnss_data.course_deg, &msg->signals[1]),
      physical_to_raw(gnss_data.position_fix, &msg->signals[2]),
      physical_to_raw(gnss_data.satellites, &msg->signals[3]),
      physical_to_raw(gnss_data.hdop, &msg->signals[4]),
  };
  can_send_message_raw32(&hcan1, msg, values);
}

void can_tx_gnss3(void) {
  const can_message_t *msg = &mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_GNSS3];
  const uint32_t values[] = {
      physical_to_raw(gnss_data.altitude_m, &msg->signals[0]),
      physical_to_raw(gnss_data.geoid_sep_m, &msg->signals[1]),
      physical_to_raw((float)0, &msg->signals[2]), // TODO: Hardcoded state.
  };
  can_send_message_raw32(&hcan1, msg, values);
}

void can_tx_quaternion(void) {
  const can_message_t *msg = &mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_QUATERNION];
  const uint32_t values[] = {
      physical_to_raw(bno085_quaternion_i, &msg->signals[0]),
      physical_to_raw(bno085_quaternion_j, &msg->signals[1]),
      physical_to_raw(bno085_quaternion_k, &msg->signals[2]),
      physical_to_raw(bno085_quaternion_real, &msg->signals[3]),
  };
  can_send_message_raw32(&hcan1, msg, values);
}

void can_tx_mag(void) {
  const can_message_t *msg =
      &mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_MAGNETOMETER];
  const uint32_t values[] = {
      physical_to_raw(bno085_mag_x, &msg->signals[0]),
      physical_to_raw(bno085_mag_y, &msg->signals[1]),
      physical_to_raw(bno085_mag_z, &msg->signals[2]),
  };
  can_send_message_raw32(&hcan1, msg, values);
}

void can_tx_gyro(void) {
  const can_message_t *msg = &mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_GYROSCOPE];
  const uint32_t values[] = {
      physical_to_raw(bno085_gyro_x, &msg->signals[0]),
      physical_to_raw(bno085_gyro_y, &msg->signals[1]),
      physical_to_raw(bno085_gyro_z, &msg->signals[2]),
  };
  can_send_message_raw32(&hcan1, msg, values);
}

void can_tx_accel(void) {
  const can_message_t *msg =
      &mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_ACCELEROMETER];
  const uint32_t values[] = {
      physical_to_raw(bno085_accel_x, &msg->signals[0]),
      physical_to_raw(bno085_accel_y, &msg->signals[1]),
      physical_to_raw(bno085_accel_z, &msg->signals[2]),
  };
  can_send_message_raw32(&hcan1, msg, values);
}

void can_tx_lin_accel(void) {
  const can_message_t *msg =
      &mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_LINEAR_ACCELEROMETER];
  const uint32_t values[] = {
      physical_to_raw(bno085_lin_accel_x, &msg->signals[0]),
      physical_to_raw(bno085_lin_accel_y, &msg->signals[1]),
      physical_to_raw(bno085_lin_accel_z, &msg->signals[2]),
  };
  can_send_message_raw32(&hcan1, msg, values);
}

void can_tx_gravity(void) {
  const can_message_t *msg =
      &mod_dbc_messages[MOMENTUM_CAN_DBC_IDX_GRAVITY_ACCELEROMETER];
  const uint32_t values[] = {
      physical_to_raw(bno085_gravity_x, &msg->signals[0]),
      physical_to_raw(bno085_gravity_y, &msg->signals[1]),
      physical_to_raw(bno085_gravity_z, &msg->signals[2]),
  };
  can_send_message_raw32(&hcan1, msg, values);
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

void comm_tx_quaternion(void) {
  printf("qi=%.3f,qj=%.3f,qk=%.3f,qr=%.3f\r\n", bno085_quaternion_i,
         bno085_quaternion_j, bno085_quaternion_k, bno085_quaternion_real);
}

void comm_tx_gyro(void) {
  printf("gx=%.3f,gy=%.3f,gz=%.3f\r\n", bno085_gyro_x, bno085_gyro_y,
         bno085_gyro_z);
}

void comm_tx_mag(void) {
  printf("mx=%.3f,my=%.3f,mz=%.3f\r\n", bno085_mag_x, bno085_mag_y,
         bno085_mag_z);
}

void comm_tx_accel(void) {
  printf("ax=%.3f,ay=%.3f,az=%.3f\r\n", bno085_accel_x, bno085_accel_y,
         bno085_accel_z);
}

void comm_tx_lin_accel(void) {
  printf("lax=%.3f,lay=%.3f,laz=%.3f\r\n", bno085_lin_accel_x,
         bno085_lin_accel_y, bno085_lin_accel_z);
}

void comm_tx_gravity(void) {
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
