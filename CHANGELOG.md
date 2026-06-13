# Changelog

---

<details markdown="1">
  <summary>Table of Contents</summary>

<!-- TOC -->
* [Changelog](#changelog)
  * [v0.1.0 (2025-08-21)](#v010--2025-08-21-)
  * [v0.2.2 (2025-12-29)](#v022--2025-12-29-)
  * [v0.2.3 (2026-01-09)](#v023--2026-01-09-)
  * [v0.2.4 (2026-01-29)](#v024--2026-01-29-)
  * [v0.3.1 (2026-03-11)](#v031--2026-03-11-)
  * [v0.3.2 (2026-04-22)](#v032--2026-04-22-)
  * [v0.4.0 (2026-04-28)](#v040--2026-04-28-)
  * [v0.4.1 (2026-05-01)](#v041--2026-05-01-)
  * [v0.4.2 (2026-05-04)](#v042--2026-05-04-)
  * [v0.4.5 (2026-05-13)](#v045--2026-05-13-)
  * [v0.4.6 (2026-06-07)](#v046--2026-06-07-)
  * [v0.4.9 (2026-06-12)](#v049--2026-06-12-)
<!-- TOC -->

</details>

---

## [v0.1.0 (2025-08-21)](https://github.com/scalpelspace/momentum/releases/tag/v0.1.0)

- Initial release.

---

## [v0.2.2 (2025-12-29)](https://github.com/scalpelspace/momentum/releases/tag/v0.2.2)

- **Additions:**
    - Add `CHANGELOG.md`.
    - Implement simple DMA driven `USART1` interface for development/debug.
- **Modifications:**
    - Modernize `.ioc` file for CubeMX `v6.15.0`.
    - Restructure files for modernized CMake toolchain and workflow.
    - Update `momentum_driver` for CAN bus logic and DBC definition upgrades.
        - CAN bus drivers now implemented by inner `can_driver` submodule.
            - Update CMakeLists.txt.
    - Update scheduler to own and utilize `TIM2` instead of `DWT`.
    - Improve and update `README.md`.
    - Initialize GNSS data specific for clearer data interpretation.

---

## [v0.2.3 (2026-01-09)](https://github.com/scalpelspace/momentum/releases/tag/v0.2.3)

- **Modifications:**
    - Fix CAN bus time sample point to 87.5% (previously 60%).

> **Post Release Notes:**
> - Error: Version macros were not properly updated, incorrectly left as
    `v0.2.2`.

---

## [v0.2.4 (2026-01-29)](https://github.com/scalpelspace/momentum/releases/tag/v0.2.4)

- **Modifications:**
    - Update `momentum_driver` for tagged release `v0.1.0`.
    - Cleanup block diagram.

---

## [v0.3.1 (2026-03-11)](https://github.com/scalpelspace/momentum/releases/tag/v0.3.1)

- **Modifications:**
    - Tighten typing in macros.
    - Update `momentum_driver` for tagged release `v0.2.0`.
        - Implements the new CAN ID scheme, `can_driver`, release `v0.3.0`.
    - Correct all incorrect references of "gps" to "gnss".

---

## [v0.3.2 (2026-04-22)](https://github.com/scalpelspace/momentum/releases/tag/v0.3.2)

- **Modifications:**
    - Minor documentation and code cleanup.
    - Move `can_id_allocatee_state_machine()` to scheduler task rather than
      direct superloop call.
    - Update `momentum_driver` for tagged release `v0.2.2`.
        - Updates `can_driver` to tagged release `v0.3.6` which has a minor CAN
          bus data decode fixes.
        - Critical DBC fixes.
    - Update `arm_gcc_build.yaml` to run on updates to submodules.
        - All submodules included (`BMP3_SensorAPI`, `momentum_driver`, `sh2`).

---

## [v0.4.0 (2026-04-28)](https://github.com/scalpelspace/momentum/releases/tag/v0.4.0)

- **Modifications:**
    - Add new `BNO085` magnetometer data collection and reporting.
        - Update `momentum_driver` for tagged release `v0.3.0`.
            - Improve naming of IMU messages following DBC changes.
            - DBC refactors present for signal encoding format (now using
              signed).

---

## [v0.4.1 (2026-05-01)](https://github.com/scalpelspace/momentum/releases/tag/v0.4.1)

- **Modifications:**
    - Update `momentum_driver` for tagged release `v0.3.2`.
        - Implement new CAN bus messages (`gnss_utc_get`,
          `gnss_utc_get_response`, `rgb_led_set`).
        - Implement automatic allocatee state machine initialization and
          re-initialization.
            - Add new configuration macro `ALLOW_CAN_NODE_ID_REASSIGNMENT` for
              Node ID reassignment permission.

---

## [v0.4.2 (2026-05-04)](https://github.com/scalpelspace/momentum/releases/tag/v0.4.2)

- **Modifications:**
    - Implement magnetometer data for Momentum SPI interface.
        - Update `momentum_driver` for tagged release `v0.3.3`.

---

## [v0.4.5 (2026-05-13)](https://github.com/scalpelspace/momentum/releases/tag/v0.4.5)

- **Additions:**
    - Add sub-second UTC sync using `SAM-M10Q` `TIMEPULSE` on PC14 EXTI.
    - Add `ublox_get_utc_ms_now()` returning UTC milliseconds since midnight.
    - Add LED task for `SAM-M10Q` GNSS status.
- **Modifications:**
    - Seed UTC seconds from parsed GGA/RMC NMEA hh:mm:ss timestamps.
    - Tick `gnss_data.hour/minute/second` at the PPS edge instead of at NMEA
      arrival, so existing CAN/UART consumers align with the actual UTC second
      boundary with CAN-frame-latency precision.
        - Reject spurious PPS edges via 1 Hz cadence gate, so older Momentum PCB
          variants without `TIMEPULSE` wired (floating PC14) cannot trip the
          validity flag from stray noise.
        - Invalidate UTC reading when no `TIMEPULSE` edge has arrived for over 2
          seconds.
    - Snapshot sync state with IRQs disabled in getter to avoid tearing across
      EXTI preemption between the `utc_seconds` and `edge_ms` reads.
    - Refactor rename `LICENSE.txt` to `LICENSE`.
    - Modernize/update `.ioc` file and code gen for CubeMX `v6.17.0`.

---

## [v0.4.6 (2026-06-07)](https://github.com/scalpelspace/momentum/releases/tag/v0.4.6)

- **Modifications:**
    - Update CAN bus message references for improved clarity.
        - Update `momentum_driver` for tagged release `v0.3.4`.

> **Post Release Notes:**
> - Error: Version macros were not properly updated, incorrectly left as
    `v0.4.5`.

---

## [v0.4.9 (2026-06-13)](https://github.com/scalpelspace/momentum/releases/tag/v0.4.9)

- **Modifications:**
    - Tighten global variable linkage and visibility.
    - Cleanup documentation and comments including `CHANGELOG.md`.
    - Add periodic state CAN message transmit with added MCU core temperature.
        - Update `momentum_driver` for tagged release `v0.3.5`.
        - Remove unused `can_tx_state()` previously defined in telemetry module.
    - Fix `SAM-M10Q` UART for timing/scheduling improvements.
        - Swap to IDLE-line only DMA, remove redundant RxCplt.
        - Expand DMA buffer to 512 bytes.
        - Swap to cache only on DMA and transmit via on scheduler task.
    - Reduce and simplify the default sensor report rates:
        - `BNO085` sensor configuration now defined via configration macros:
            - Quaternion (game vector): 100 Hz.
            - Gyroscope: 100 Hz.
            - Accelerometer: 100 Hz.
            - Magnetometer: 25 Hz.
        - `BMP390` scheduler task: 25 Hz (set via configration macro).
        - `SAM-M10Q` scheduler task: 10 Hz with staggered tick.
    - Rebalance NVIC preemption priorities.
        - `WS2812B` LED reduced to lower priority (3).
        - Swap `SAM-M10Q` and `BNO085` priority, previously 1 and 2, now 2 and 1
          respectively.
