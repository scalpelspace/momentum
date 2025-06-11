/*******************************************************************************
 * @file    w25qxx_hal_spi.c
 * @brief   W25Qxx functions: abstracting STM32 HAL: SPI.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "w25qxx_hal_spi.h"

/** Definitions. **************************************************************/

#define W25Q_CS_LOW()                                                          \
  HAL_GPIO_WritePin(W25QXX_CS_GPIO_PORT, W25QXX_CS_GPIO_PIN, GPIO_PIN_RESET)
#define W25Q_CS_HIGH()                                                         \
  HAL_GPIO_WritePin(W25QXX_CS_GPIO_PORT, W25QXX_CS_GPIO_PIN, GPIO_PIN_SET)

#define W25Q_CMD_PAGE_PROGRAM 0x02
#define W25Q_CMD_READ_DATA 0x03
#define W25Q_CMD_READ_STATUS1 0x05
#define W25Q_CMD_WRITE_ENABLE 0x06
#define W25Q_CMD_FAST_READ_DATA 0x0B
#define W25Q_CMD_SECTOR_ERASE 0x20
#define W25Q_CMD_BLOCK_ERASE_32K 0x52
#define W25Q_CMD_JEDEC_ID 0x9F
#define W25Q_CMD_RELEASE_PD 0xAB
#define W25Q_CMD_CHIP_ERASE 0xC7
#define W25Q_CMD_BLOCK_ERASE_64K 0xD8

/** Private functions. ********************************************************/

static HAL_StatusTypeDef w25q_write_enable(void) {
  uint8_t cmd = W25Q_CMD_WRITE_ENABLE;
  W25Q_CS_LOW();
  HAL_StatusTypeDef st = HAL_SPI_Transmit(&W25QXX_HSPI, &cmd, 1, HAL_MAX_DELAY);
  W25Q_CS_HIGH();
  return st;
}

static HAL_StatusTypeDef w25q_wait_busy(void) {
  uint8_t cmd = W25Q_CMD_READ_STATUS1;
  uint8_t status;
  do {
    W25Q_CS_LOW();
    HAL_SPI_Transmit(&W25QXX_HSPI, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&W25QXX_HSPI, &status, 1, HAL_MAX_DELAY);
    W25Q_CS_HIGH();
  } while (status & 0x01);
  return HAL_OK;
}

/** Public functions. *********************************************************/

HAL_StatusTypeDef w25q_init(void) {
  // Release deep power-down.
  uint8_t cmd = W25Q_CMD_RELEASE_PD;
  W25Q_CS_LOW();
  HAL_SPI_Transmit(&W25QXX_HSPI, &cmd, 1, HAL_MAX_DELAY);
  W25Q_CS_HIGH();
  HAL_Delay(1);
  return HAL_OK;
}

HAL_StatusTypeDef w25q_read_jedec(uint8_t *manuf, uint8_t *mem_type,
                                  uint8_t *capacity) {
  uint8_t tx[4] = {W25Q_CMD_JEDEC_ID, 0, 0, 0};
  uint8_t rx[4] = {0};
  W25Q_CS_LOW();
  HAL_StatusTypeDef st =
      HAL_SPI_TransmitReceive(&W25QXX_HSPI, tx, rx, 4, HAL_MAX_DELAY);
  W25Q_CS_HIGH();
  if (st == HAL_OK) {
    *manuf = rx[1];
    *mem_type = rx[2];
    *capacity = rx[3];
  }
  return st;
}

HAL_StatusTypeDef w25q_read_data(uint8_t *pBuf, uint32_t addr, uint32_t len) {
  uint8_t cmd[4] = {W25Q_CMD_READ_DATA, (uint8_t)(addr >> 16),
                    (uint8_t)(addr >> 8), (uint8_t)(addr)};
  W25Q_CS_LOW();
  HAL_StatusTypeDef st = HAL_SPI_Transmit(&W25QXX_HSPI, cmd, 4, HAL_MAX_DELAY);
  if (st == HAL_OK) {
    st = HAL_SPI_Receive(&W25QXX_HSPI, pBuf, len, HAL_MAX_DELAY);
  }
  W25Q_CS_HIGH();
  return st;
}

HAL_StatusTypeDef w25q_fast_read_data(uint8_t *pBuf, uint32_t addr,
                                      uint32_t len) {
  uint8_t cmd[5] = {W25Q_CMD_FAST_READ_DATA, (uint8_t)(addr >> 16),
                    (uint8_t)(addr >> 8), (uint8_t)(addr), 0xFF};
  W25Q_CS_LOW();
  HAL_StatusTypeDef st = HAL_SPI_Transmit(&W25QXX_HSPI, cmd, 5, HAL_MAX_DELAY);
  if (st == HAL_OK) {
    st = HAL_SPI_Receive(&W25QXX_HSPI, pBuf, len, HAL_MAX_DELAY);
  }
  W25Q_CS_HIGH();
  return st;
}

HAL_StatusTypeDef w25q_page_program(const uint8_t *pBuf, uint32_t addr,
                                    uint16_t len) {
  if (len > W25Q_PAGE_SIZE)
    return HAL_ERROR;
  HAL_StatusTypeDef st = w25q_write_enable();
  if (st != HAL_OK)
    return st;

  uint8_t cmd[4] = {W25Q_CMD_PAGE_PROGRAM, (uint8_t)(addr >> 16),
                    (uint8_t)(addr >> 8), (uint8_t)(addr)};
  W25Q_CS_LOW();
  st = HAL_SPI_Transmit(&W25QXX_HSPI, cmd, 4, HAL_MAX_DELAY);
  if (st == HAL_OK) {
    st = HAL_SPI_Transmit(&W25QXX_HSPI, (uint8_t *)pBuf, len, HAL_MAX_DELAY);
  }
  W25Q_CS_HIGH();
  if (st == HAL_OK) {
    st = w25q_wait_busy();
  }
  return st;
}

HAL_StatusTypeDef w25q_sector_erase(uint32_t addr) {
  HAL_StatusTypeDef st = w25q_write_enable();
  if (st != HAL_OK)
    return st;
  uint8_t cmd[4] = {W25Q_CMD_SECTOR_ERASE, (uint8_t)(addr >> 16),
                    (uint8_t)(addr >> 8), (uint8_t)(addr)};
  W25Q_CS_LOW();
  st = HAL_SPI_Transmit(&W25QXX_HSPI, cmd, 4, HAL_MAX_DELAY);
  W25Q_CS_HIGH();
  if (st == HAL_OK) {
    st = w25q_wait_busy();
  }
  return st;
}

HAL_StatusTypeDef w25q_block_erase_32k(uint32_t addr) {
  HAL_StatusTypeDef st = w25q_write_enable();
  if (st != HAL_OK)
    return st;
  uint8_t cmd[4] = {W25Q_CMD_BLOCK_ERASE_32K, (uint8_t)(addr >> 16),
                    (uint8_t)(addr >> 8), (uint8_t)(addr)};
  W25Q_CS_LOW();
  st = HAL_SPI_Transmit(&W25QXX_HSPI, cmd, 4, HAL_MAX_DELAY);
  W25Q_CS_HIGH();
  if (st == HAL_OK) {
    st = w25q_wait_busy();
  }
  return st;
}

HAL_StatusTypeDef w25q_block_erase_64k(uint32_t addr) {
  HAL_StatusTypeDef st = w25q_write_enable();
  if (st != HAL_OK)
    return st;
  uint8_t cmd[4] = {W25Q_CMD_BLOCK_ERASE_64K, (uint8_t)(addr >> 16),
                    (uint8_t)(addr >> 8), (uint8_t)(addr)};
  W25Q_CS_LOW();
  st = HAL_SPI_Transmit(&W25QXX_HSPI, cmd, 4, HAL_MAX_DELAY);
  W25Q_CS_HIGH();
  if (st == HAL_OK) {
    st = w25q_wait_busy();
  }
  return st;
}

HAL_StatusTypeDef w25q_chip_erase(void) {
  HAL_StatusTypeDef st = w25q_write_enable();
  if (st != HAL_OK)
    return st;
  uint8_t cmd = W25Q_CMD_CHIP_ERASE;
  W25Q_CS_LOW();
  st = HAL_SPI_Transmit(&W25QXX_HSPI, &cmd, 1, HAL_MAX_DELAY);
  W25Q_CS_HIGH();
  if (st == HAL_OK) {
    // Full chip erase can take a while (~6-10 s).
    st = w25q_wait_busy();
  }
  return st;
}
