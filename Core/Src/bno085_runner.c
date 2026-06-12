/*******************************************************************************
 * @file bno085_runner.c
 * @brief BNO085 SH2 runner: init, start reports and event handling.
 *******************************************************************************
 * @note
 * Developed using https://github.com/ceva-dsp/sh2-demo-nucleo as reference.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "bno085_runner.h"

#include "configuration.h"
#if defined(MOMENTUM_FULL_CAN_TELEMETRY) ||                                    \
    defined(MOMENTUM_FULL_COMM_TELEMETRY)
#include "telemetry.h"
#endif

/** Private variables. ********************************************************/

static sh2_Hal_t *sh2_hal_instance = 0;
static bool reset_occurred = false;

/** Public variables. *********************************************************/

sensor_config_t sensor_config[SH2_MAX_SENSOR_ID] = {
    // Fused orientation quaternion.
    // 100 Hz.
    {SH2_GAME_ROTATION_VECTOR, {.reportInterval_us = 10000}},

    // Calibrated gyroscope data.
    // 100 Hz.
    {SH2_GYROSCOPE_CALIBRATED, {.reportInterval_us = 10000}},

    // Calibrated accelerometer data on X, Y and Z axes.
    // 100 Hz.
    {SH2_ACCELEROMETER, {.reportInterval_us = 10000}},

    // Calibrated magnetometer data.
    // 25 Hz.
    {SH2_MAGNETIC_FIELD_CALIBRATED, {.reportInterval_us = 40000}},
};

float bno085_quaternion_i = 0;
float bno085_quaternion_j = 0;
float bno085_quaternion_k = 0;
float bno085_quaternion_real = 0;
float bno085_quaternion_accuracy_rad = 0;
float bno085_quaternion_accuracy_deg = 0;
float bno085_gyro_x = 0;
float bno085_gyro_y = 0;
float bno085_gyro_z = 0;
float bno085_mag_x = 0;
float bno085_mag_y = 0;
float bno085_mag_z = 0;
float bno085_accel_x = 0;
float bno085_accel_y = 0;
float bno085_accel_z = 0;
float bno085_lin_accel_x = 0;
float bno085_lin_accel_y = 0;
float bno085_lin_accel_z = 0;
float bno085_gravity_x = 0;
float bno085_gravity_y = 0;
float bno085_gravity_z = 0;

/** Private functions. ********************************************************/

void sh2_error_handler(const int status) {
  switch (status) {
  case SH2_OK:
    break;
  case SH2_ERR:
    // TODO: Error handling for general error.
    break;
  case SH2_ERR_BAD_PARAM:
    // TODO: Error handling for bad parameter to an API call.
    break;
  case SH2_ERR_OP_IN_PROGRESS:
    // TODO: Error handling for operation in progress.
    break;
  case SH2_ERR_IO:
    // TODO: Error handling for error communicating with hub.
    break;
  case SH2_ERR_HUB:
    // TODO: Error handling for error reported by hub.
    break;
  case SH2_ERR_TIMEOUT:
    // TODO: Error handling for operation timed out.
    break;
  default:
    break;
  }
}

/**
 * @brief Configure periodic reports.
 *
 * @note See section `5.1 Sensor Metadata` in
 * `1000-3625 - SH-2 Reference Manual v1.4` for all possible metadata records.
 */
static void start_reports() {
  for (int n = 0; n < ARRAY_LEN(sensor_config); n++) {
    const int status = sh2_setSensorConfig(sensor_config[n].sensorId,
                                           &sensor_config[n].config);
    if (status != SH2_OK) {
      sh2_error_handler(status);
    }
  }
}

/**
 * @brief Handle non-sensor events from the sensor hub.
 */
static void general_event_handler(void *cookie, sh2_AsyncEvent_t *pEvent) {
  (void)cookie; // Unused.

  // If we see a reset, set a flag so that sensors will be reconfigured.
  if (pEvent->eventId == SH2_RESET) {
    reset_occurred = true;

  } else if (pEvent->eventId == SH2_SHTP_EVENT) {
    // TODO: IMPLEMENT EVENT HANDLER pEvent->shtpEvent.

  } else if (pEvent->eventId == SH2_GET_FEATURE_RESP) {
    // TODO: IMPLEMENT EVENT HANDLER for pEvent->sh2SensorConfigResp.sensorId.
  }
}

/**
 * @brief Handle sensor events from the sensor hub.
 */
static void sensor_report_handler(void *cookie, sh2_SensorEvent_t *pEvent) {
  (void)cookie; // Unused.

  sh2_SensorValue_t value;
  int sh2_status = sh2_decodeSensorEvent(&value, pEvent);

  if (sh2_status != SH2_OK) {
    sh2_error_handler(sh2_status);
    return;
  }

  // Get HAL associated SH2 timer value via:
  // double timestamp_sec = (double)value.timestamp / 1000000.0;

  switch (value.sensorId) {
  case SH2_GAME_ROTATION_VECTOR:
    bno085_quaternion_i = value.un.gameRotationVector.i;
    bno085_quaternion_j = value.un.gameRotationVector.j;
    bno085_quaternion_k = value.un.gameRotationVector.k;
    bno085_quaternion_real = value.un.gameRotationVector.real;
    bno085_quaternion_accuracy_rad = 0.0f;
    bno085_quaternion_accuracy_deg = 0.0f;

#ifdef MOMENTUM_FULL_CAN_TELEMETRY
    can_tx_quaternion();
#endif
#ifdef MOMENTUM_FULL_COMM_TELEMETRY
    comm_tx_quaternion();
#endif
    break;

  case SH2_GYROSCOPE_CALIBRATED:
    bno085_gyro_x = value.un.gyroscope.x;
    bno085_gyro_y = value.un.gyroscope.y;
    bno085_gyro_z = value.un.gyroscope.z;

#ifdef MOMENTUM_FULL_CAN_TELEMETRY
    can_tx_gyro();
#endif
#ifdef MOMENTUM_FULL_COMM_TELEMETRY
    comm_tx_gyro();
#endif
    break;

  case SH2_MAGNETIC_FIELD_CALIBRATED:
    bno085_mag_x = value.un.magneticField.x;
    bno085_mag_y = value.un.magneticField.y;
    bno085_mag_z = value.un.magneticField.z;

#ifdef MOMENTUM_FULL_CAN_TELEMETRY
    can_tx_mag();
#endif
#ifdef MOMENTUM_FULL_COMM_TELEMETRY
    comm_tx_mag();
#endif
    break;

  case SH2_ACCELEROMETER:
    bno085_accel_x = value.un.accelerometer.x;
    bno085_accel_y = value.un.accelerometer.y;
    bno085_accel_z = value.un.accelerometer.z;

#ifdef MOMENTUM_FULL_CAN_TELEMETRY
    can_tx_accel();
#endif
#ifdef MOMENTUM_FULL_COMM_TELEMETRY
    comm_tx_accel();
#endif
    break;

  case SH2_LINEAR_ACCELERATION:
    bno085_lin_accel_x = value.un.linearAcceleration.x;
    bno085_lin_accel_y = value.un.linearAcceleration.y;
    bno085_lin_accel_z = value.un.linearAcceleration.z;

#ifdef MOMENTUM_FULL_CAN_TELEMETRY
    can_tx_lin_accel();
#endif
#ifdef MOMENTUM_FULL_COMM_TELEMETRY
    comm_tx_lin_accel();
#endif
    break;

  case SH2_GRAVITY:
    bno085_gravity_x = value.un.gravity.x;
    bno085_gravity_y = value.un.gravity.y;
    bno085_gravity_z = value.un.gravity.z;

#ifdef MOMENTUM_FULL_CAN_TELEMETRY
    can_tx_gravity();
#endif
#ifdef MOMENTUM_FULL_COMM_TELEMETRY
    comm_tx_gravity();
#endif
    break;

  default: // Handle unknown sensor reports.
    break;
  }
}

/** Public functions. *********************************************************/

void bno085_init(void) {
  // Create SH2 HAL instance.
  sh2_hal_instance = sh2_hal_init();

  // Open SH2 interface (also registers non-sensor event handler.)
  const int status = sh2_open(sh2_hal_instance, general_event_handler, NULL);
  if (status != SH2_OK) {
    sh2_error_handler(status);
  }

  // Register sensor listener.
  sh2_setSensorCallback(sensor_report_handler, NULL);

  // Reset now possible it since sensor reports will be started.
  reset_occurred = false;

  // Start the flow of sensor reports.
  start_reports();
}

void bno085_reset(void) {
  HAL_GPIO_WritePin(SH2_RSTN_PORT, SH2_RSTN_PIN, GPIO_PIN_RESET);
  HAL_Delay(5); // Hold reset for 5 ms.
  HAL_GPIO_WritePin(SH2_RSTN_PORT, SH2_RSTN_PIN, GPIO_PIN_SET);
}

void bno085_run(void) {
  // Get HAL associated SH2 timer value via:
  // uint32_t now = sh2_hal_instance->getTimeUs(sh2_hal_instance);

  if (reset_occurred) {
    // Restart the flow of sensor reports.
    reset_occurred = false;
    start_reports();
  }

  // Service the sensor hub.
  // Sensor reports and event processing handled by callbacks.
  sh2_service();
}
