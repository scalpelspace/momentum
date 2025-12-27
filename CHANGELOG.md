# Changelog

---

<details markdown="1">
  <summary>Table of Contents</summary>

<!-- TOC -->
* [Changelog](#changelog)
  * [v0.1.0 (2025-08-21)](#v010--2025-08-21-)
  * [v0.2.2 (WIP)](#v022--wip-)
<!-- TOC -->

</details>

---

## [v0.1.0 (2025-08-21)](https://github.com/scalpelspace/momentum/releases/tag/v0.1.0)

- Initial release.

---

## [v0.2.2 (WIP)](https://github.com/scalpelspace/momentum/releases/tag/v0.2.2)

- **Additions:**
    - Add `CHANGELOG.md`.
    - Implement simple DMA driven USART1 interface for development/debug.
- **Modifications:**
    - Modernize `.ioc` file for CubeMX `v6.15.0`.
    - Restructure files for modernized CMake toolchain and workflow.
    - Update `momentum_driver` for CAN bus logic and DBC definition upgrades.
        - CAN bus drivers now implemented by inner `can_driver` submodule.
            - Update CMakeLists.txt.
    - Update scheduler to own and utilize `TIM2` instead of `DWT`.
    - Improve and update `README.md`.
        - Reduce CP2102N USB-UART docs to centralize docs.
    - Rewrite UART-to-USB comm modules to a simple debug data transmit.
