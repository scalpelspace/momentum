/*******************************************************************************
 * @file    w25qxx_hal_spi.h
 * @brief   W25Qxx functions: abstracting STM32 HAL: SPI.
 *******************************************************************************
 */

#ifndef MOMENTUM__W25QXX_H
#define MOMENTUM__W25QXX_H

/** Includes. *****************************************************************/

#include "stm32l4xx_hal.h"

/** STM32 port and pin configs. ***********************************************/

extern SPI_HandleTypeDef hspi3;

// SPI.
#define W25QXX_HSPI hspi3
#define W25QXX_CS_GPIO_PORT GPIOA
#define W25QXX_CS_GPIO_PIN GPIO_PIN_15

/** Definitions. **************************************************************/

#define W25Q_PAGE_SIZE 256U
#define W25Q_SECTOR_SIZE 4096U

/** Public functions. *********************************************************/

HAL_StatusTypeDef w25q_init(void);
static HAL_StatusTypeDef w25q_wait_busy(void);
HAL_StatusTypeDef w25q_read_jedec(uint8_t *manufacturer, uint8_t *mem_type,
                                  uint8_t *capacity);
HAL_StatusTypeDef w25q_read_data(uint8_t *buffer, uint32_t addr, uint32_t len);
HAL_StatusTypeDef w25q_fast_read_data(uint8_t *buffer, uint32_t addr,
                                      uint32_t len);
HAL_StatusTypeDef w25q_page_program(const uint8_t *buffer, uint32_t addr,
                                    uint16_t len);
HAL_StatusTypeDef w25q_sector_erase(uint32_t addr);
HAL_StatusTypeDef w25q_block_erase_32k(uint32_t addr);
HAL_StatusTypeDef w25q_block_erase_64k(uint32_t addr);
HAL_StatusTypeDef w25q_chip_erase(void);

#endif
