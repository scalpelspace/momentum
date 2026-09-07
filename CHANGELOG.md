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
  * [v0.4.9 (2026-06-13)](#v049--2026-06-13-)
  * [v0.4.10 (2026-06-24)](#v0410--2026-06-24-)
  * [v0.4.11 (2026-07-04)](#v0411--2026-07-04-)
  * [v0.5.0 (2026-07-06)](#v050--2026-07-06-)
  * [v0.5.1 (2026-07-09)](#v051--2026-07-09-)
  * [v0.5.2 (2026-07-10)](#v052--2026-07-10-)
  * [v0.5.3 (2026-08-02)](#v053--2026-08-02-)
  * [v0.5.4 (2026-08-16)](#v054--2026-08-16-)
  * [v0.6.0 (2026-09-01)](#v060--2026-09-01-)
  * [v0.6.1 (TBD)](#v061--tbd-)
<!-- TOC -->

</details>

---

## [v0.1.0 (2025-08-21)](https://github.com/scalpelspace/momentum/releases/tag/v0.1.0)

- Initial release.

---

## [v0.2.2 (2025-12-29)](https://github.com/scalpelspace/momentum/releases/tag/v0.2.2)

- Add `CHANGELOG.md`.
- Implement simple DMA driven `USART1` interface for development/debug.
- Modernize `.ioc` file for CubeMX v6.15.0.
- Restructure files for modernized CMake toolchain and workflow.
- Update `momentum_driver` for CAN bus logic and DBC definition upgrades.
    - CAN bus drivers now implemented by inner `can_driver` submodule.
        - Update CMakeLists.txt.
- Update scheduler to own and utilize `TIM2` instead of `DWT`.
- Improve and update `README.md`.
- Initialize GNSS data specific for clearer data interpretation.

---

## [v0.2.3 (2026-01-09)](https://github.com/scalpelspace/momentum/releases/tag/v0.2.3)

- Fix CAN bus time sample point to 87.5% (previously 60%).

> **Post Release Notes:**
> - Error: Version macros were not properly updated, incorrectly left as v0.2.2.

---

## [v0.2.4 (2026-01-29)](https://github.com/scalpelspace/momentum/releases/tag/v0.2.4)

- Update `momentum_driver` for tagged release v0.1.0.
- Cleanup block diagram.

---

## [v0.3.1 (2026-03-11)](https://github.com/scalpelspace/momentum/releases/tag/v0.3.1)

- Tighten typing in macros.
- Update `momentum_driver` for tagged release v0.2.0.
    - Implements the new CAN ID scheme, `can_driver`, release v0.3.0.
- Correct all incorrect references of "gps" to "gnss".

---

## [v0.3.2 (2026-04-22)](https://github.com/scalpelspace/momentum/releases/tag/v0.3.2)

- Minor documentation and code cleanup.
- Move `can_id_allocatee_state_machine()` to scheduler task rather than direct
  superloop call.
- Update `momentum_driver` for tagged release v0.2.2.
    - Updates `can_driver` to tagged release v0.3.6 which has a minor CAN bus
      data decode fixes.
    - Critical DBC fixes.
- Update `arm_gcc_build.yaml` to run on updates to submodules.
    - All submodules included (`BMP3_SensorAPI`, `momentum_driver`, `sh2`).

---

## [v0.4.0 (2026-04-28)](https://github.com/scalpelspace/momentum/releases/tag/v0.4.0)

- Add new BNO085 magnetometer data collection and reporting.
    - Update `momentum_driver` for tagged release v0.3.0.
        - Improve naming of IMU messages following DBC changes.
        - DBC refactors present for signal encoding format (now using signed).

---

## [v0.4.1 (2026-05-01)](https://github.com/scalpelspace/momentum/releases/tag/v0.4.1)

- Update `momentum_driver` for tagged release v0.3.2.
    - Implement new CAN bus messages (`gnss_utc_get`, `gnss_utc_get_response`,
      `rgb_led_set`).
    - Implement automatic allocatee state machine initialization and
      re-initialization.
        - Add new configuration macro `ALLOW_CAN_NODE_ID_REASSIGNMENT` for Node
          ID reassignment permission.

---

## [v0.4.2 (2026-05-04)](https://github.com/scalpelspace/momentum/releases/tag/v0.4.2)

- Implement magnetometer data for Momentum SPI interface.
    - Update `momentum_driver` for tagged release v0.3.3.

---

## [v0.4.5 (2026-05-13)](https://github.com/scalpelspace/momentum/releases/tag/v0.4.5)

- Add sub-second UTC sync using SAM-M10Q `TIMEPULSE` on PC14 EXTI.
- Add `ublox_get_utc_ms_now()` returning UTC milliseconds since midnight.
- Add LED task for SAM-M10Q GNSS status.
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
  `EXTI` preemption between the `utc_seconds` and `edge_ms` reads.
- Refactor rename `LICENSE.txt` to `LICENSE`.
- Modernize/update `.ioc` file and code gen for CubeMX v6.17.0.

---

## [v0.4.6 (2026-06-07)](https://github.com/scalpelspace/momentum/releases/tag/v0.4.6)

- Update CAN bus message references for improved clarity.
    - Update `momentum_driver` for tagged release v0.3.4.

> **Post Release Notes:**
> - Error: Version macros were not properly updated, incorrectly left as v0.4.5.

---

## [v0.4.9 (2026-06-13)](https://github.com/scalpelspace/momentum/releases/tag/v0.4.9)

- Tighten global variable linkage and visibility.
- Cleanup documentation and comments including `CHANGELOG.md`.
- Add periodic state CAN message transmit with added MCU core temperature.
    - Update `momentum_driver` for tagged release v0.3.5.
    - Remove unused `can_tx_state()` previously defined in telemetry module.
- Fix SAM-M10Q UART for timing/scheduling improvements.
    - Swap to IDLE-line only DMA, remove redundant RxCplt.
    - Expand DMA buffer to 512 bytes.
    - Swap to cache only on DMA and transmit via on scheduler task.
- Reduce and simplify the default sensor report rates:
    - BNO085 sensor configuration now defined via configration macros:
        - Quaternion (game vector): 100 Hz.
        - Gyroscope: 100 Hz.
        - Accelerometer: 100 Hz.
        - Magnetometer: 25 Hz.
    - BMP390 scheduler task: 25 Hz (set via configration macro).
    - SAM-M10Q scheduler task: 10 Hz with staggered tick.
- Rebalance NVIC preemption priorities.
    - WS2812B LED reduced to lower priority (3).
    - Swap SAM-M10Q and BNO085 priority, previously 1 and 2, now 2 and 1
      respectively.

---

## [v0.4.10 (2026-06-24)](https://github.com/scalpelspace/momentum/releases/tag/v0.4.10)

- Add `uid` retrival in comm UART/Serial Interface.
    - Update UART/Serial Interface docs.

---

## [v0.4.11 (2026-07-04)](https://github.com/scalpelspace/momentum/releases/tag/v0.4.11)

- Tighten function linkage and visibility.
- Guard the shared CAN TX header/mailbox with interrupt disable to prevent
  ISR/main-loop races.
- Change DLC mismatch handling to skip only that entry instead of aborting the
  whole dispatch loop.
- Refactor CAN telemetry transmit to use const pointer message reference and
  direct signal initialization.

---

## [v0.5.0 (2026-07-06)](https://github.com/scalpelspace/momentum/releases/tag/v0.5.0)

- Update for `momentum_driver` tagged release v0.4.0.
    - Update `can_send_message_raw32()` for muxed signal CAN transmit.

---

## [v0.5.1 (2026-07-09)](https://github.com/scalpelspace/momentum/releases/tag/v0.5.1)

- Cleanup `CHANGELOG.md` for formatting and syntax consistency.
- Enable `AutoBusOff` and `AutoRetransmission` via CubeMX.
- Update `momentum_driver` to tagged release v0.4.1.

---

## [v0.5.2 (2026-07-10)](https://github.com/scalpelspace/momentum/releases/tag/v0.5.2)

- Update scheduler to run `can_id_allocatee_state_machine` at 20 ms period
  (previously 250 ms).
- Update `momentum_driver` to tagged version v0.4.2.

---

## [v0.5.3 (2026-08-02)](https://github.com/scalpelspace/momentum/releases/tag/v0.5.3)

- Cleanup peripheral checking to compare `->Instance` instead of handle pointers
  in HAL callbacks.
    - Match callbacks to their peripheral by register instance (via new
      `*_INSTANCE` macros) rather than by handle address, and normalize all
      checkers to the early-return guard-clause form.
- Fix WS2812B PWM DMA memory data width to match the `uint16_t` DMA buffer.
    - Previously was BYTE, corrected to HALFWORD.

---

## [v0.5.4 (2026-08-16)](https://github.com/scalpelspace/momentum/releases/tag/v0.5.4)

- Rename `ALLOW_CAN_NODE_ID_REASSIGNMENT` to `ALLOW_CAN_NODE_ID_ALLOCATION`, now
  acts as a single switch for the CAN ID allocation protocol.
    - Undefined: the node never participates in allocation and operates on
      `DEFAULT_CAN_NODE_ID` for its lifetime. The allocatee scheduler task and
      the allocatee receive callbacks are compiled out.
    - Defined: unchanged behaviour, the allocatee runs and restarts after every
      assignment.
- Support pre-compile time CAN Node IDs via `DEFAULT_CAN_NODE_ID`.
    - A non-zero `DEFAULT_CAN_NODE_ID` is now applied to the DBC message IDs in
      `can_db_init()`. Previously the value only set `can_node_id` and the DBC
      was left unpatched, so the node transmitted under the authored ID.
    - Add a compile time range check rejecting IDs outside `[0, 30]`.
- Reorganize `README.md` docs.

---

## [v0.6.0 (2026-09-01)](https://github.com/scalpelspace/momentum/releases/tag/v0.6.0)

- Update `momentum_driver` to tagged version v0.5.0.
- Carry the `ALLOW_CAN_NODE_ID_ALLOCATION` configuration forward onto the new
  `can_alloc_mode_t` advertised by the allocatee.
    - Undefined now maps to `CAN_ALLOC_MODE_NOT_REASSIGNABLE` instead of
      compiling the protocol out. The node still advertises, so the allocator
      reserves `DEFAULT_CAN_NODE_ID` rather than handing it to another node.
      Assignments are refused inside the allocatee.
    - Defined maps to `CAN_ALLOC_MODE_REASSIGNABLE`, unchanged behaviour.
    - The allocatee scheduler task and the allocatee receive callbacks are no
      longer compiled out, they are required in both modes to answer DISCOVER.
    - Add a compile time check rejecting `DEFAULT_CAN_NODE_ID` of `0` while
      `ALLOW_CAN_NODE_ID_ALLOCATION` is undefined, a node that refuses
      reassignment has to hold a real Node ID to advertise.
- Seed `allocatee_config_t::node_id` with the Node ID currently held rather than
  `DEFAULT_CAN_NODE_ID`, so a restarted allocatee re-advertises the ID it was
  assigned.
- Remove the allocatee restart from `allocatee_complete()`.
    - The allocatee now returns to awaiting discovery on its own after ACKing.
      Restarting reset the held Node ID back to the configured seed, so the node
      would advertise as unassigned on the next session.

---

## [v0.6.1 (TBD)](https://github.com/scalpelspace/momentum/releases/tag/v0.6.1)

- Update UART (comm) to report 6 decimal places for latitude and longitude.
    - Previously capped to 3 decimal places.
- Add support for `help` command.
- Improve and update `README.md`.
- Clean up and remove unnecessary includes.
- Update the format of GNSS3 UART (comm) telemetry output to include GNSS date
  and time.
- Remove unused `comm_tx_rtc()` function.
