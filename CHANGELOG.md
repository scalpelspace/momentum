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
