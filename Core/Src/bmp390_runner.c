/*******************************************************************************
 * @file bmp390_runner.c
 * @brief BMP390 runner: init and data reading.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "bmp390_runner.h"

/** Public variables. *********************************************************/

float bmp390_temperature;
float bmp390_pressure;

/** Private variables. ********************************************************/

struct bmp3_dev dev;
struct bmp3_fifo_settings fifo_settings = {0};
uint8_t fifo_data[FIFO_MAX_SIZE];
struct bmp3_fifo_data fifo = {0};

/** Private functions. ********************************************************/

void bmp3_error_handler(const int8_t status) {
  switch (status) {
  case BMP3_OK:
    break;
  case BMP3_E_NULL_PTR:
    // TODO: Error handling for null pointer.
    break;
  case BMP3_E_COMM_FAIL:
    // TODO: Error handling for communication failure.
    break;
  case BMP3_E_INVALID_LEN:
    // TODO: Error handling for incorrect length parameter.
    break;
  case BMP3_E_DEV_NOT_FOUND:
    // TODO: Error handling for device not found.
    break;
  case BMP3_E_CONFIGURATION_ERR:
    // TODO: Error handling for configuration Error.
    break;
  case BMP3_W_SENSOR_NOT_ENABLED:
    // TODO: Error handling for warning when Sensor not enabled.
    break;
  case BMP3_W_INVALID_FIFO_REQ_FRAME_CNT:
    // TODO: Error handling for warning Fifo watermark level not in limit.
    break;
  default:
    // TODO: Error handling for unknown error code.
    break;
  }
}

/** Public functions. *********************************************************/

int8_t bmp390_init(void) {
  const uint16_t settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN |
                                BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS |
                                BMP3_SEL_ODR;
  const uint16_t settings_fifo = BMP3_SEL_FIFO_MODE | BMP3_SEL_FIFO_PRESS_EN |
                                 BMP3_SEL_FIFO_TEMP_EN | BMP3_SEL_FIFO_FULL_EN |
                                 BMP3_SEL_FIFO_DOWN_SAMPLING |
                                 BMP3_SEL_FIFO_FILTER_EN;
  struct bmp3_settings settings = {0};

  int8_t hal_init_status = bmp3_interface_init(&dev, BMP3_I2C_INTF);
  if (hal_init_status != BMP3_OK) {
    bmp3_error_handler(hal_init_status);
    return hal_init_status;
  }

  int8_t bmp3_init_status = bmp3_init(&dev);
  if (bmp3_init_status != BMP3_OK) {
    bmp3_error_handler(bmp3_init_status);
    return bmp3_init_status;
  }

  // Initialize FIFO options.
  fifo_settings.mode = BMP3_ENABLE;
  fifo_settings.press_en = BMP3_ENABLE;
  fifo_settings.temp_en = BMP3_ENABLE;
  fifo_settings.filter_en = BMP3_ENABLE;
  fifo_settings.down_sampling = BMP3_FIFO_NO_SUBSAMPLING;

  // Initialize FIFO.
  fifo.buffer = fifo_data;
  fifo.req_frames = FIFO_FRAME_COUNT;

  // Initialize general settings.
  settings.press_en = BMP3_ENABLE; // Pressure.
  settings.temp_en = BMP3_ENABLE;  // Temperature.

  // Datasheet referenced and recommended values.
  settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
  settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
  settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
  settings.odr_filter.odr = BMP3_ODR_100_HZ;

  int8_t sensor_settings_status =
      bmp3_set_sensor_settings(settings_sel, &settings, &dev);
  if (sensor_settings_status != BMP3_OK) {
    bmp3_error_handler(sensor_settings_status);
    return sensor_settings_status;
  }

  // Set normal operation mode.
  settings.op_mode = BMP3_MODE_NORMAL;
  bmp3_set_op_mode(&settings, &dev);

  int8_t fifo_settings_status =
      bmp3_set_fifo_settings(settings_fifo, &fifo_settings, &dev);
  if (fifo_settings_status != BMP3_OK) {
    bmp3_error_handler(fifo_settings_status);
    return fifo_settings_status;
  }

  return BMP3_OK;
}

void bmp390_get_data(void) {
  uint16_t fifo_length = 0;
  struct bmp3_status status = {{0}};
  struct bmp3_data fifo_p_t_data[FIFO_MAX_SIZE];
  float temperature_sum = 0;
  float pressure_sum = 0;

  int8_t result = bmp3_get_status(&status, &dev);

  if (result == BMP3_OK) {
    bmp3_get_fifo_length(&fifo_length, &dev);
    bmp3_get_fifo_data(&fifo, &fifo_settings, &dev);

    // Read status register again to clear FIFO Full interrupt status.
    result = bmp3_get_status(&status, &dev);

    if (result == BMP3_OK) {
      bmp3_extract_fifo_data(fifo_p_t_data, &fifo, &dev);

      // Use moving average of frames.
      if (fifo.parsed_frames > 0) {
        for (uint8_t index = 0; index < fifo.parsed_frames; index++) {
          temperature_sum += (float)fifo_p_t_data[index].temperature;
          pressure_sum += (float)fifo_p_t_data[index].pressure;
        }
        bmp390_temperature = temperature_sum / (float)fifo.parsed_frames;
        bmp390_pressure = pressure_sum / (float)fifo.parsed_frames;

        if (status.intr.fifo_full == BMP3_ENABLE) {
          bmp3_fifo_flush(&dev); // Flush FIFO if full.
        }
      }
    }
  } else {
    bmp3_error_handler(result);
    bmp3_soft_reset(&dev);
    bmp390_init();
  }
}
