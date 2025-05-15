/*******************************************************************************
 * @file bno085_runner.h
 * @brief BNO085 SH2 runner: init, start reports and event handling.
 *******************************************************************************
 * @note
 * Developed using https://github.com/ceva-dsp/sh2-demo-nucleo as reference.
 *******************************************************************************
 */

#ifndef MOMENTUM__BNO085_RUNNER_H
#define MOMENTUM__BNO085_RUNNER_H

/** Includes. *****************************************************************/

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "sh2_hal_spi.h"
#include <stdio.h>

/** Definitions. **************************************************************/

#define RAD_TO_DEG (180.0 / 3.14159265358)

/** Public variables. *********************************************************/

extern float bno085_quaternion_i;
extern float bno085_quaternion_j;
extern float bno085_quaternion_k;
extern float bno085_quaternion_real;
extern float bno085_quaternion_accuracy_rad;
extern float bno085_quaternion_accuracy_deg;
extern float bno085_gyro_x;
extern float bno085_gyro_y;
extern float bno085_gyro_z;
extern float bno085_accel_x;
extern float bno085_accel_y;
extern float bno085_accel_z;
extern float bno085_lin_accel_x;
extern float bno085_lin_accel_y;
extern float bno085_lin_accel_z;
extern float bno085_gravity_x;
extern float bno085_gravity_y;
extern float bno085_gravity_z;

/** Public functions. *********************************************************/

/**
 * @brief Initialize BNO085 with SH2 driver.
 */
void bno085_init(void);

/**
 * @brief Reset BNO085.
 */
void bno085_reset(void);

/**
 * @brief Run sensor run cycle.
 */
void bno085_run(void);

#endif
