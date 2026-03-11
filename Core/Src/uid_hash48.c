/*******************************************************************************
 * @file uid_hash48.c
 * @brief STM32L432KC UID 48-bit hash generation.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "uid_hash48.h"
#include "stm32l4xx_hal.h"
#include <string.h>

/** Public functions. *********************************************************/

uint64_t get_uid_hash48(void) {
  const uint32_t uid0 = HAL_GetUIDw0();
  const uint32_t uid1 = HAL_GetUIDw1();
  const uint32_t uid2 = HAL_GetUIDw2();

  // Combine into 64-bit accumulator.
  const uint64_t x = ((uint64_t)uid0 << 32) | (uint64_t)uid1;
  const uint64_t y = (uint64_t)uid2;

  // 64-bit mix (split mix-style avalanche).
  uint64_t h = x ^ (y << 21) ^ (y >> 7);

  h ^= h >> 33;
  h *= UINT64_C(0xFF51AFD7ED558CCD);
  h ^= h >> 33;
  h *= UINT64_C(0xC4CEB9FE1A85EC53);
  h ^= h >> 33;

  // Return lower 48 bits.
  return h & UINT64_C(0x0000FFFFFFFFFFFF);
}

void get_uid_hash48_parts(uint16_t *uid0, uint16_t *uid1, uint16_t *uid2) {
  const uint64_t hash48 = get_uid_hash48();

  if (uid0)
    *uid0 = (uint16_t)(hash48 & 0xFFFFu);
  if (uid1)
    *uid1 = (uint16_t)((hash48 >> 16) & 0xFFFFu);
  if (uid2)
    *uid2 = (uint16_t)((hash48 >> 32) & 0xFFFFu);
}
