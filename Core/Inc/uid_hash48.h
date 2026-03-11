/*******************************************************************************
 * @file uid_hash48.h
 * @brief STM32L432KC UID 48-bit hash generation.
 *******************************************************************************
 */

#ifndef PWM_NODE__UID_HASH48_H
#define PWM_NODE__UID_HASH48_H

/** Includes. *****************************************************************/

#include <stdint.h>

/** Public functions. *********************************************************/

/**
 * @brief Determine 48-bit STM32L432KC UID hash.
 *
 * @return 48-bit STM32L432KC UID hash.
 */
uint64_t get_uid_hash48(void);

/**
 * @brief Determine 3x 16-bit (48-bit split) STM32L432KC UID hash.
 *
 * @param uid0 UID hash48, 16-bit segment 1 of 3 (bits 0..15 of 48-bit hash).
 * @param uid1 UID hash48, 16-bit segment 2 of 3 (bits 16..31 of 48-bit hash).
 * @param uid2 UID hash48, 16-bit segment 3 of 3 (bits 32..47 of 48-bit hash).
 */
void get_uid_hash48_parts(uint16_t *uid0, uint16_t *uid1, uint16_t *uid2);

#endif
