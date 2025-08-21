/*******************************************************************************
 * @file bmp5_hal_i2c.c
 * @brief BMP5 functions: abstracting STM32 HAL: I2C.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "bmp5_hal_i2c.h"
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
static uint32_t bmp5_time_now_us(void) {
  return __HAL_TIM_GET_COUNTER(&BMP5_HTIM);
}

/** Public functions. *********************************************************/

BMP5_INTF_RET_TYPE bmp5_interface_init(struct bmp5_dev *bmp5, uint8_t intf) {
  int8_t result = BMP5_OK;

  if (bmp5 != NULL) {
    // TODO: NOTE: Bus configuration hardcoded locked to I2C here!
    if (intf == BMP5_I2C_INTF) {
      device_address = BMP5_I2C_ADDRESS;
      bmp5->read = bmp5_i2c_read;
      bmp5->write = bmp5_i2c_write;
      bmp5->intf = BMP5_I2C_INTF;
    } else {
      return BMP5_E_COM_FAIL;
    }

    // Start hardware (timer).
    HAL_TIM_Base_Start(&BMP5_HTIM);

    bmp5->delay_us = bmp5_delay_us;
    bmp5->intf_ptr = &device_address;

  } else {
    result = BMP5_E_NULL_PTR;
  }

  return result;
}

BMP5_INTF_RET_TYPE bmp5_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                 uint32_t len, void *intf_ptr) {
  uint8_t device_addr = *(uint8_t *)intf_ptr << 1;
  (void)intf_ptr;
  HAL_StatusTypeDef status = HAL_ERROR;

  if (HAL_I2C_GetState(&BMP5_HI2C) == HAL_I2C_STATE_READY) {
    status = HAL_I2C_Mem_Read(&BMP5_HI2C, device_addr, reg_addr, 1, reg_data,
                              len, HAL_MAX_DELAY);
  }

  if (status != HAL_OK) {
    return BMP5_E_COM_FAIL;
  }
  return BMP5_INTF_RET_SUCCESS;
}

BMP5_INTF_RET_TYPE bmp5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                                  uint32_t len, void *intf_ptr) {
  uint8_t device_addr = *(uint8_t *)intf_ptr << 1;
  (void)intf_ptr;
  HAL_StatusTypeDef status = HAL_ERROR;

  if (HAL_I2C_GetState(&BMP5_HI2C) == HAL_I2C_STATE_READY) {
    status = HAL_I2C_Mem_Write(&BMP5_HI2C, device_addr, reg_addr, 1,
                               (uint8_t *)reg_data, len, HAL_MAX_DELAY);
  }

  if (status != HAL_OK) {
    return BMP5_E_COM_FAIL;
  }
  return BMP5_INTF_RET_SUCCESS;
}

void bmp5_delay_us(uint32_t period, void *intf_ptr) {
  (void)intf_ptr;

  volatile uint32_t now = bmp5_time_now_us();
  const uint32_t start = now;
  while ((now - start) < period) {
    now = bmp5_time_now_us();
  }
}
