/*******************************************************************************
 * @file bmp3_hal_i2c.c
 * @brief BMP3 functions: abstracting STM32 HAL: I2C.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "bmp3_hal_i2c.h"
#include <stdint.h>
#include <stdio.h>

/** Private variable. *********************************************************/

static uint8_t device_address; // Device I2C address.

/** Private functions. ********************************************************/

/**
 * @brief Get the current time in us.
 *
 * TODO: NOTE: Currently, no timer overflow handling!
 */
static uint32_t bmp3_time_now_us(void) {
  return __HAL_TIM_GET_COUNTER(&BMP3_HTIM);
}

/** Public functions. *********************************************************/

BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev *bmp3, uint8_t intf) {
  int8_t result = BMP3_OK;

  if (bmp3 != NULL) {
    // TODO: NOTE: Bus configuration hardcoded locked to I2C here!
    if (intf == BMP3_I2C_INTF) {
      device_address = BMP3_I2C_ADDRESS;
      bmp3->read = bmp3_i2c_read;
      bmp3->write = bmp3_i2c_write;
      bmp3->intf = BMP3_I2C_INTF;
    } else {
      return BMP3_ERR_FATAL;
    }

    // Start hardware (timer).
    HAL_TIM_Base_Start(&BMP3_HTIM);

    bmp3->delay_us = bmp3_delay_us;
    bmp3->intf_ptr = &device_address;

  } else {
    result = BMP3_E_NULL_PTR;
  }

  return result;
}

BMP3_INTF_RET_TYPE bmp3_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                 uint32_t len, void *intf_ptr) {
  uint8_t device_addr = *(uint8_t *)intf_ptr << 1;
  (void)intf_ptr;
  HAL_StatusTypeDef status = HAL_ERROR;

  if (HAL_I2C_GetState(&BMP3_HI2C) == HAL_I2C_STATE_READY) {
    status = HAL_I2C_Mem_Read(&BMP3_HI2C, device_addr, reg_addr, 1, reg_data,
                              len, HAL_MAX_DELAY);
  }

  if (status != HAL_OK) {
    return BMP3_ERR_FATAL;
  }
  return BMP3_INTF_RET_SUCCESS;
}

BMP3_INTF_RET_TYPE bmp3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                                  uint32_t len, void *intf_ptr) {
  uint8_t device_addr = *(uint8_t *)intf_ptr << 1;
  (void)intf_ptr;
  HAL_StatusTypeDef status = HAL_ERROR;

  if (HAL_I2C_GetState(&BMP3_HI2C) == HAL_I2C_STATE_READY) {
    status = HAL_I2C_Mem_Write(&BMP3_HI2C, device_addr, reg_addr, 1,
                               (uint8_t *)reg_data, len, HAL_MAX_DELAY);
  }

  if (status != HAL_OK) {
    return BMP3_ERR_FATAL;
  }
  return BMP3_INTF_RET_SUCCESS;
}

void bmp3_delay_us(uint32_t period, void *intf_ptr) {
  (void)intf_ptr;

  volatile uint32_t now = bmp3_time_now_us();
  const uint32_t start = now;
  while ((now - start) < period) {
    now = bmp3_time_now_us();
  }
}
