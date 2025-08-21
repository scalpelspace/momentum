/*******************************************************************************
 * @file bmp5_hal_i2c.h
 * @brief BMP5 functions: abstracting STM32 HAL: I2C.
 *******************************************************************************
 */

#ifndef MOMENTUM__BMP5_HAL_I2C_H
#define MOMENTUM__BMP5_HAL_I2C_H

/** Includes. *****************************************************************/

#include "bmp5.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"

/** STM32 port and pin configs. ***********************************************/

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;

// I2C.
#define BMP5_HI2C hi2c1

// Timer for signals.
#define BMP5_HTIM htim2

/** BMP5xx SDO I2C address selection. *****************************************/

// GPIO/BMP5xx pin state.
#define BMP5_SDO GPIO_PIN_RESET

// Macro to automatically find device address.
#define BMP5_I2C_ADDRESS                                                       \
  ((BMP5_SDO == GPIO_PIN_RESET) ? BMP5_I2C_ADDR_PRIM : BMP5_I2C_ADDR_SEC)
// SDO to GND    = slave address 01000110 (0x46) = BMP5_I2C_ADDR_PRIM.
// SDO to V_DDIO = slave address 01000111 (0x47) = BMP5_I2C_ADDR_SEC.

/** Public functions. *********************************************************/

/**
 * @brief STM32 HAL abstraction initialization.
 *
 * @param bmp5 Structure instance of bmp5_dev.
 * @param intf Interface selection parameter.
 *
 * @return Status of execution.
 * @retval == 0 -> Success.
 * @retval < 0  -> Failure info.
 */
BMP5_INTF_RET_TYPE bmp5_interface_init(struct bmp5_dev *bmp5, uint8_t intf);

/**
 * @brief Read the sensor's registers through I2C bus.
 *
 * @param reg_addr Register address.
 * @param reg_data Pointer to the data buffer to store the read data.
 * @param len Number of bytes to read.
 * @param intf_ptr Interface pointer.
 *
 * @return Status of execution.
 * @retval == BMP5_INTF_RET_SUCCESS -> Success.
 * @retval != BMP5_INTF_RET_SUCCESS -> Failure Info.
 */
BMP5_INTF_RET_TYPE bmp5_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                 uint32_t len, void *intf_ptr);

/**
 * @brief Writing to the sensor's registers through I2C bus.
 *
 * @param reg_addr Register address.
 * @param reg_data Pointer to the data buffer whose value is to be written.
 * @param len Number of bytes to write.
 * @param intf_ptr Interface pointer.
 *
 * @return Status of execution.
 * @retval == BMP5_INTF_RET_SUCCESS -> Success.
 * @retval != BMP5_INTF_RET_SUCCESS -> Failure info.
 */
BMP5_INTF_RET_TYPE bmp5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                                  uint32_t len, void *intf_ptr);

/**
 * @brief Blocking delay function for required time in microseconds.
 *
 * @param period Time in microseconds.
 * @param intf_ptr Interface pointer.
 */
void bmp5_delay_us(uint32_t period, void *intf_ptr);

#endif
