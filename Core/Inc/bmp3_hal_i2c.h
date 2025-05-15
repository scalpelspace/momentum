/*******************************************************************************
 * @file bmp3_hal_i2c.h
 * @brief BMP3 functions: abstracting STM32 HAL: I2C.
 *******************************************************************************
 */

#ifndef MOMENTUM__BMP3_HAL_I2C_H
#define MOMENTUM__BMP3_HAL_I2C_H

/** Includes. *****************************************************************/

#include "bmp3.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"

/** STM32 port and pin configs. ***********************************************/

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;

// I2C.
#define BMP3_HI2C hi2c1

// Timer for signals.
#define BMP3_HTIM htim2

/** BMP390 SDO I2C address selection. *****************************************/

// GPIO/BMP390 pin state.
#define BMP3_SDO GPIO_PIN_RESET

// Macro to automatically find device address.
#define BMP3_I2C_ADDRESS                                                       \
  ((BMP3_SDO == GPIO_PIN_RESET) ? BMP3_ADDR_I2C_PRIM : BMP3_ADDR_I2C_SEC)
// SDO to GND    = slave address 1110110 (0x76) = BMP3_ADDR_I2C_PRI.
// SDO to V_DDIO = slave address 1110111 (0x77) = BMP3_ADDR_I2C_SEC.

/** Public functions. *********************************************************/

/**
 * @brief STM32 HAL abstraction initialization.
 *
 * @param bmp3 Structure instance of bmp3_dev.
 * @param intf Interface selection parameter.
 *
 * @return Status of execution.
 * @retval == 0 -> Success.
 * @retval < 0  -> Failure info.
 */
BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev *bmp3, uint8_t intf);

/**
 * @brief Read the sensor's registers through I2C bus.
 *
 * @param reg_addr Register address.
 * @param reg_data Pointer to the data buffer to store the read data.
 * @param len Number of bytes to read.
 * @param intf_ptr Interface pointer.
 *
 * @return Status of execution.
 * @retval == BMP3_INTF_RET_SUCCESS -> Success.
 * @retval != BMP3_INTF_RET_SUCCESS -> Failure Info.
 */
BMP3_INTF_RET_TYPE bmp3_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
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
 * @retval == BMP3_INTF_RET_SUCCESS -> Success.
 * @retval != BMP3_INTF_RET_SUCCESS -> Failure info.
 */
BMP3_INTF_RET_TYPE bmp3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                                  uint32_t len, void *intf_ptr);

/**
 * @brief Blocking delay function for required time in microseconds.
 *
 * @param period Time in microseconds.
 * @param intf_ptr Interface pointer.
 */
void bmp3_delay_us(uint32_t period, void *intf_ptr);

#endif
