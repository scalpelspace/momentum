/*******************************************************************************
 * @file bmp5xx_runner.c
 * @brief BMP5xx runner: init and data reading.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "bmp5xx_runner.h"

#include "configuration.h"
#ifdef MOMENTUM_FULL_CAN_TELEMETRY
#include "telemetry.h"
#endif

/** Public variables. *********************************************************/

float bmp5xx_temperature;
float bmp5xx_pressure;

/** Private variables. ********************************************************/

struct bmp5_dev dev_5;
struct bmp5_osr_odr_press_config osr_odr_press_cfg = {0};

/** Private functions. ********************************************************/

void bmp5_error_handler(const int8_t status) {
  switch (status) {
  case BMP5_OK:
    break;
  case BMP5_E_NULL_PTR:
    // TODO: Error handling for null pointer.
    break;
  case BMP5_E_COM_FAIL:
    // TODO: Error handling for communication failure.
    break;
  case BMP5_E_DEV_NOT_FOUND:
    // TODO: Error handling for device not found.
    break;
  default:
    // TODO: Error handling for unknown error code.
    break;
  }
}

/** Private functions. ********************************************************/

static int8_t set_config(void) {
  int8_t rslt;
  struct bmp5_iir_config set_iir_cfg;
  struct bmp5_int_source_select int_source_select;

  rslt = bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, &dev_5);

  if (rslt == BMP5_OK) {
    // Get default ODR.
    rslt = bmp5_get_osr_odr_press_config(&osr_odr_press_cfg, &dev_5);

    if (rslt == BMP5_OK) {
      // Set ODR as 120 Hz.
      osr_odr_press_cfg.odr = BMP5_ODR_120_HZ;

      // Enable pressure.
      osr_odr_press_cfg.press_en = BMP5_ENABLE;

      // Set Over-sampling rate with respect to ODR.
      osr_odr_press_cfg.osr_t = BMP5_OVERSAMPLING_64X;
      osr_odr_press_cfg.osr_p = BMP5_OVERSAMPLING_4X;

      rslt = bmp5_set_osr_odr_press_config(&osr_odr_press_cfg, &dev_5);
    }

    if (rslt == BMP5_OK) {
      set_iir_cfg.set_iir_t = BMP5_IIR_FILTER_COEFF_1;
      set_iir_cfg.set_iir_p = BMP5_IIR_FILTER_COEFF_1;
      set_iir_cfg.shdw_set_iir_t = BMP5_ENABLE;
      set_iir_cfg.shdw_set_iir_p = BMP5_ENABLE;

      rslt = bmp5_set_iir_config(&set_iir_cfg, &dev_5);
    }

    // Set power mode as normal.
    rslt = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, &dev_5);
  }

  return rslt;
}

/** Public functions. *********************************************************/

int8_t bmp5xx_init(void) {
  int8_t hal_init_status = bmp5_interface_init(&dev_5, BMP5_I2C_INTF);
  if (hal_init_status != BMP5_OK) {
    bmp5_error_handler(hal_init_status);
    return hal_init_status;
  }

  int8_t bmp5_init_status = bmp5_init(&dev_5);
  if (bmp5_init_status != BMP5_OK) {
    bmp5_error_handler(bmp5_init_status);
    return bmp5_init_status;
  }

  bmp5_init_status = set_config();
  if (bmp5_init_status != BMP5_OK) {
    bmp5_error_handler(bmp5_init_status);
  }

  return bmp5_init_status;
}

int8_t bmp5xx_get_data(void) {
  int8_t rslt = 0;
  uint8_t int_status;
  struct bmp5_sensor_data sensor_data;

  rslt = bmp5_get_interrupt_status(&int_status, &dev_5);

  if (int_status & BMP5_INT_ASSERTED_DRDY) {
    rslt = bmp5_get_sensor_data(&sensor_data, &osr_odr_press_cfg, &dev_5);

    if (rslt == BMP5_OK) {
      bmp5xx_pressure = sensor_data.pressure;
      bmp5xx_temperature = sensor_data.temperature;

#ifdef MOMENTUM_FULL_CAN_TELEMETRY
      can_tx_barometric();
#endif
    }
  }

  return rslt;
}
