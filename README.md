# momentum

![arm_gcc_build](https://github.com/scalpelspace/momentum/actions/workflows/arm_gcc_build.yaml/badge.svg)

STM32L432KC microcontroller firmware for `momentum_pcb`.

- GNSS, IMU and barometer sensor hub compatible with Uno-style microcontroller
  boards via SPI and CAN bus.

---

<details markdown="1">
  <summary>Table of Contents</summary>

<!-- TOC -->
* [momentum](#momentum)
  * [1 Overview](#1-overview)
    * [1.1 Bill of Materials (BOM)](#11-bill-of-materials-bom)
    * [1.2 Block Diagram](#12-block-diagram)
    * [1.3 Pin Configurations](#13-pin-configurations)
    * [1.4 Clock Configurations](#14-clock-configurations)
  * [2 USB Interface via CP2102N (USB-to-UART)](#2-usb-interface-via-cp2102n-usb-to-uart)
    * [2.1 Data Line Activity LEDs](#21-data-line-activity-leds)
  * [3 Serial Peripheral Interface (SPI)](#3-serial-peripheral-interface-spi)
  * [4 BNO086 9-DOF IMU](#4-bno086-9-dof-imu)
    * [4.1 Background](#41-background)
    * [4.2 Serial Peripheral Interface (SPI)](#42-serial-peripheral-interface-spi)
      * [4.2.1 Full-Duplex vs. Half-Duplex](#421-full-duplex-vs-half-duplex)
      * [4.2.2 Clock Polarity, Phase and Modes](#422-clock-polarity-phase-and-modes)
      * [4.2.3 Clock Rate](#423-clock-rate)
      * [4.2.4 Direct Memory Access (DMA)](#424-direct-memory-access-dma)
    * [4.3 General-Purpose Input/Output (GPIO) Output](#43-general-purpose-inputoutput-gpio-output)
    * [4.4 Timer](#44-timer)
      * [4.4.1 Timer Prescaler Calculation](#441-timer-prescaler-calculation)
    * [4.5 Nested Vectored Interrupt Controller (NVIC)](#45-nested-vectored-interrupt-controller-nvic)
    * [4.6 BNO086 Driver](#46-bno086-driver)
      * [4.6.1 SH2 Set Reorientation Quaternion](#461-sh2-set-reorientation-quaternion)
  * [5 BMP390 Barometric Pressure Sensor](#5-bmp390-barometric-pressure-sensor)
    * [5.1 Background](#51-background)
    * [5.2 Inter-Integrated Circuit (I2C)](#52-inter-integrated-circuit-i2c)
    * [5.3 Timer](#53-timer)
    * [5.4 BMP390 Driver](#54-bmp390-driver)
  * [6 TJA1057BTK CAN Bus Transceiver](#6-tja1057btk-can-bus-transceiver)
    * [6.1 Background](#61-background)
    * [6.2 Controller Area Network (CAN)](#62-controller-area-network-can)
      * [6.2.1 Bit Time Calculation](#621-bit-time-calculation)
      * [6.2.2 Nested Vectored Interrupt Controller (NVIC)](#622-nested-vectored-interrupt-controller-nvic)
    * [6.3 CAN High-Level Driver](#63-can-high-level-driver)
    * [6.4 CAN DBC and Low-Level Driver](#64-can-dbc-and-low-level-driver)
  * [7 SAM-M10Q RF Receiver Galileo, GLONASS, GPS](#7-sam-m10q-rf-receiver-galileo-glonass-gps)
    * [7.1 Background](#71-background)
    * [7.2 Universal Synchronous/Asynchronous Receiver/Transmitter (USART)](#72-universal-synchronousasynchronous-receivertransmitter-usart)
      * [7.2.1 Direct Memory Access (DMA)](#721-direct-memory-access-dma)
      * [7.2.2 Nested Vectored Interrupt Controller (NVIC)](#722-nested-vectored-interrupt-controller-nvic)
    * [7.3 SAM-M10Q Driver](#73-sam-m10q-driver)
  * [8 WS2812B PWM Addressable RGB LED](#8-ws2812b-pwm-addressable-rgb-led)
    * [8.1 Clocks](#81-clocks)
    * [8.2 Pulse Width Modulation (PWM) Timer](#82-pulse-width-modulation-pwm-timer)
      * [8.2.1 Timer Calculations](#821-timer-calculations)
    * [8.3 Direct Memory Access (DMA)](#83-direct-memory-access-dma)
    * [8.4 Nested Vectored Interrupt Controller (NVIC)](#84-nested-vectored-interrupt-controller-nvic)
    * [8.5 WS2812B Driver](#85-ws2812b-driver)
      * [8.5.1 PWM Duty Cycle Calculations](#851-pwm-duty-cycle-calculations)
      * [8.5.2 Reset Code Time Periods Calculation](#852-reset-code-time-periods-calculation)
  * [9 Real Time Clock (RTC)](#9-real-time-clock-rtc)
    * [9.1 RTC Driver](#91-rtc-driver)
  * [10 Shared Low-Level Software Features](#10-shared-low-level-software-features)
    * [10.1 Callbacks](#101-callbacks)
  * [11 Third-Party Licenses](#11-third-party-licenses)
<!-- TOC -->

</details>

---

## 1 Overview

### 1.1 Bill of Materials (BOM)

| Manufacturer Part Number | Manufacturer            | Description                       | Quantity | Notes |
|--------------------------|-------------------------|-----------------------------------|---------:|-------|
| STM32L432KC              | STMicroelectronics      | 32-bit MCU                        |        1 |       |
| CP2102N-A02-GQFN24R      | Silicon Labs            | USB 2.0 to UART Interface         |        1 |       |
| BNO086                   | CEVA Technologies, Inc. | 9-DOF IMU                         |        1 |       |
| BMP390                   | Bosch Sensortec         | Barometric Pressure Sensor        |        1 |       |
| TJA1057BTK               | NXP USA Inc.            | CAN Bus Transceiver               |        1 |       |
| SAM-M10Q                 | u-blox                  | RF Receiver Galileo, GLONASS, GPS |        1 |       |
| WS2812B                  | (Various)               | PWM Addressable RGB LED           |        1 |       |

### 1.2 Block Diagram

![momentum.drawio.png](docs/momentum.drawio.png)

> Drawio file here: [momentum.drawio](docs/momentum.drawio).

### 1.3 Pin Configurations

<details markdown="1">
  <summary>CubeMX Pinout</summary>

![CubeMX Pinout.png](docs/CubeMX%20Pinout.png)

</details>

<details markdown="1">
  <summary>Pin & Peripherals Table</summary>

| STM32L432KC | Peripheral              | Config                           | Connection                        | Notes                                           |
|-------------|-------------------------|----------------------------------|-----------------------------------|-------------------------------------------------|
| PA14        | `SYS_JTCK-SWCLK`        |                                  | TC2050 SWD Pin 4: `SWCLK`         |                                                 |
| PA13        | `SYS_JTMS-SWDIO`        |                                  | TC2050 SWD Pin 2: `SWDIO`         |                                                 |
|             | `TIM2_CH1`              | PWM no output                    |                                   | Scheduler, BNO086 SH2 and BMP390 BMP3 timer.    |
| PA5         | `SPI1_SCK`              |                                  | BNO086 Pin 19: `H_SCL/SCK/RX`     |                                                 |
| PA4         | `GPIO_Output` (SPI1 CS) | Set high                         | BNO086 Pin 18: `H_CSN`            |                                                 |
| PA6         | `SPI1_MISO`             |                                  | BNO086 Pin 20: `H_SDA/H_MISO/TX`  |                                                 |
| PA7         | `SPI1_MOSI`             |                                  | BNO086 Pin 17: `SA0/H_MOSI`       |                                                 |
| PB0         | `GPIO_EXTI0`            | Pull-up, falling edge            | BNO086 Pin 14: `H_INTN`           |                                                 |
| PB1         | `GPIO_Output`           | Set high                         | BNO086 Pin 6: `PS0/Wake`          | Pull low to trigger wake.                       |
|             |                         | Hardware pull-up                 | BNO086 Pin 5: `PS1`               |                                                 |
| PA1         | `GPIO_Output`           | Set high                         | BNO086 Pin 11: `NRST`             | Pull low to reset.                              |
| PB6         | `I2C1_SCL`              |                                  | BMP390 Pin 2: `SCK`               |                                                 |
| PB7         | `I2C1_SDA`              |                                  | BMP390 Pin 4: `SDI`               |                                                 |
| PA3         | `USART2_RX`             | 9600 bps (-> 115200 in software) | SAM-M10Q Pin 13: `TXD`            | Starts as 9600 bps to match the u-blox default. |
| PA2         | `USART2_TX`             | 9600 bps (-> 115200 in software) | SAM-M10Q Pin 14: `RXD`            | Starts as 9600 bps to match the u-blox default. |
| PC15        | `GPIO_Output`           |                                  | SAM-M10Q Pin 18: `RESET_N`        | Pull low to reset (>= 1 ms).                    |
| PA10        | `USART1_RX`             | 115200 bps                       | CP2102N-A02-GQFN24R Pin 20: `TXD` |                                                 |
| PA9         | `USART1_TX`             | 115200 bps                       | CP2102N-A02-GQFN24R Pin 21: `RXD` |                                                 |
| PA11        | `CAN1_RX`               |                                  | TJA1057BTK Pin 1: `TXD`           |                                                 |
| PA12        | `CAN1_TX`               |                                  | TJA1057BTK Pin 4: `RXD`           |                                                 |
| PA8         | `TIM1_CH1`              | PWM Generation CH1               | WS2812B Pin: `DIN`                | DIN pin number depends on IC variant.           |
| PB3         | `SPI3_SCK`              |                                  | SPI interface: `SCK`              |                                                 |
| PA15        | `SPI3_NSS`              | Pull-up, set high                | SPI interface: `SS`               |                                                 |
| PB4         | `SPI3_MISO`             |                                  | SPI interface: `MISO`             |                                                 |
| PB5         | `SPI3_MOSI`             |                                  | SPI interface: `MOSI`             |                                                 |

</details>

### 1.4 Clock Configurations

```
8 MHz High Speed External (HSE)
↓
Phase-Locked Loop Main (PLLM)
↓
80 MHz SYSCLK
↓
80 MHz HCLK
↓
 → 80 MHz APB1 (Maxed) → 80 MHz APB1 Timer
 → 80 MHz APB2 (Maxed) → 80 MHz APB2 Timer
```

---

## 2 USB Interface via CP2102N (USB-to-UART)

Utilizing the USB interface via the CP2102N requires Silicon
Labs's [VCP CP210x USB to UART Bridge VCP Drivers](https://www.silabs.com/developer-tools/usb-to-uart-bridge-vcp-drivers).

### 2.1 Data Line Activity LEDs

Direct from the manufacturer/suppliers the CP2102N's LED GPIOs are not enabled.
To enable the LED GPIOs, the CP2102N undergoes additional configuration
programming via Silicon
Labs's [Simplicity Studio Software](https://www.silabs.com/developer-tools/simplicity-studio).

![cp2102n_leds_config.png](docs/cp2102n_leds_config.png)

`GPIO0` and `GPIO1`'s `Alternative Function` need to be configured as
`TX Toggle` and `RX Toggle` respectively as shown in the picture above.

---

## 3 Serial Peripheral Interface (SPI)

SPI configurations:

- CPOL = 0.
- CPHA = 0.
- Hardware peripheral select (NSS) enabled.

Low level SPI communication drivers can be found
here: [`momentum_driver`](https://github.com/scalpelspace/momentum_driver).

---

## 4 BNO086 9-DOF IMU

> **Note:** Momentum was originally designed for the BNO085, however hardware
> files were updated to reflect use of the newer BNO086. Firmware is cross
> compatible for both the BNO085/6, however source files maintain the use of
> the "BNO085" naming.

9-axis Inertial Measurement Unit (IMU) combining an accelerometer, gyroscope,
and magnetometer, based on Bosch Sensortec's BNO080 hardware, with sensor fusion
firmware developed by CEVA, Inc. (formerly Hillcrest Laboratories).
(Non-standard) I2C and SPI capable.

> Utilized reference documents:
> 1. `1000-3535 - Sensor Hub Transport Protocol v1.8`
> 2. `1000-3600 - SH-2 SHTP Reference Manual v1.5_1`
> 3. `1000-3625 - SH-2 Reference Manual v1.4`
> 4. `1000-3918 - BNO080 Migration_1`
> 5. `1000-3927 - BNO080 Datasheet v1.6`
> 6. `1000-4044 - BNO080-BNO085 Sensor Calibration Procedure v1.3`
> 7. `1000-4045 - App Note - BNO080-BNO085 Tare Function Usage Guide_1`
> 8. `HillcrestLabs BNO080-085 DataSheet_C`

### 4.1 Background

The BNO086 runs the same hardware as the BNO080, however runs custom Sensor Hub
2 (SH-2) firmware to reduce development overhead on features related to sensor
fusion and optimization. SH-2 is designed around the Sensor Hub Transport
Protocol (SHTP), which runs on SPI, I2C, etc.

### 4.2 Serial Peripheral Interface (SPI)

In an SPI setup, there is always one controller (master) connected to one or
more peripherals (slaves). The controller controls the communication by
generating a clock signal (SCK) and selecting which slave to communicate with
using the Chip Select (CS) line. Data is exchanged between the controller and
peripheral(s) over two data lines: COPI/MOSI and CIPO or MISO.

Basic pinouts:

1. Clock (SCK)
    - The clock signal generated by the master device that synchronizes data
      transfer in SPI communication.

2. Chip Select (CS) or Slave Select (NSS)
    - A signal used to select a specific slave device in SPI communication. When
      the CS line is active (usually low), the selected slave device is enabled
      to communicate with the master.

3. Controller Out Peripheral In (COPI) or Master Out Slave In (MOSI)
    - The data line used to transfer data from the master device to the slave
      device. The master outputs data on this line, which the slave reads.

4. Controller In Peripheral Out (CIPO) or Master In Slave Out (MISO)
    - The data line used to transfer data from the slave device to the master
      device. The slave outputs data on this line, which the master reads.

Since firmware describes the role of the central STM32L432KC controller, the SPI
configuration is controller (master).

#### 4.2.1 Full-Duplex vs. Half-Duplex

Full-Duplex: Data can be sent and received simultaneously.

Half-Duplex: Data is either sent or received at any given time, not both
simultaneously.

For the purposes of bidirectional communication full-duplex mode is selected.

#### 4.2.2 Clock Polarity, Phase and Modes

CPOL (Clock Polarity): determines the idle state of the clock signal (SCK).

- CPOL = 0: The clock is low (0) when idle.
- CPOL = 1: The clock is high (1) when idle.

CPHA (Clock Phase): determines when data is sampled relative to the clock
signal.

- CPHA = 0: Data is sampled on the leading (1st) clock edge.
- CPHA = 1: Data is sampled on the trailing (2nd) clock edge.

SPI Modes (Combination of CPOL and CPHA):

| Mode | CPOL | CPHA | SCK idle state | Data captured on               | Data output on |
|:----:|:----:|:----:|:--------------:|--------------------------------|----------------|
|  0   |  0   |  0   |    Low (0)     | Rising edge of SCK (1st edge)  | Falling edge   |
|  1   |  0   |  1   |    Low (0)     | Falling edge of SCK (2nd edge) | Rising edge    |
|  2   |  1   |  0   |    High (1)    | Falling edge of SCK (1st edge) | Rising edge    |
|  3   |  1   |  1   |    High (1)    | Rising edge of SCK (2nd edge)  | Falling edge   |

The datasheet specifies the use of CPOL = 1 (high) and CPHA = 1 (2nd edge).

#### 4.2.3 Clock Rate

As specified in the datasheet, the maximum SPI clock rate is 3 MHz. Given that
SPI1 runs on the APB2 bus clock (80 MHz), and the prescaler values are powers of
2 (2, 4, 8, etc.):

$$PSC = \frac{Source}{Target} - 1 = \frac{ 80 \space \mathrm{MHz} }{ 3 \space \mathrm{MHz} } - 1 = 25.7$$

PSC = 32 is used (powers of 2).

$$Clock = \frac{Source}{PSC} = \frac{ 80 \space \mathrm{MHz} }{ 32 } = 2.5 \space \mathrm{MHz}$$

Final clock rate is 2.5 MHz.

#### 4.2.4 Direct Memory Access (DMA)

DMA is enabled for both SPI1 RX and TX in order to reduce interrupt utilization.

`SPI1_RX` `DMA2 Stream3`:

- Direction: `Peripheral to Memory`.
- Mode: `Normal`.
- Peripheral Increment Address: `Disabled`.
- Memory Increment Address: `Enabled`.
- (Both Peripheral and Memory) Data Width: `Byte`.
- Use FIFO: `Disabled`.

`SPI1_TX` `DMA1 Stream3`:

- Direction: `Memory to Peripheral`.
- Mode: `Normal`.
- Peripheral Increment Address: `Disabled`.
- Memory Increment Address: `Enabled`.
- (Both Peripheral and Memory) Data Width: `Byte`.
- Use FIFO: `Disabled`.

### 4.3 General-Purpose Input/Output (GPIO) Output

3 GPIO output pins are used to control pins: PS0/Wake, PS1 and NRST. These pins
manage SPI/I2C switching, the SPI configuration and reset. Theoretically, some
of these can be pulled low via hardware and it would still work. However, as
suggested by the official SH2 driver struct and for flexibility purposes, all 3
pins are set for their own GPIO output pins.

### 4.4 Timer

TIM2 is configured to be used for timing operations (1 µs time base) in the SH2
SHTP drivers.

#### 4.4.1 Timer Prescaler Calculation

TIM2 runs based on the APB1 timer clocks which are set to 80 MHz. The
prescaler (PSC) must be calculated accordingly to achieve a 1 µs (1 MHz) time
base. In other words, aiming for 1 tick = 1 µs.

$$PSC = \frac{Source}{Target} - 1 = \frac{ 80 \space \mathrm{MHz} }{ 1 \space \mathrm{MHz} } - 1 = 79$$

### 4.5 Nested Vectored Interrupt Controller (NVIC)

`GPIO_EXTI0`is configured for the `INTN` pin of the BNO086 to trigger an MCU
response:

- External Interrupt Mode with Falling edge trigger detection.
- Pull-up.

### 4.6 BNO086 Driver

Submodule: [sh2](Core/sh2).

Source: [github.com/ceva-dsp/sh2](https://github.com/ceva-dsp/sh2).

STM32 HAL abstraction and runner functions:

1. [sh2_hal_spi.h](Core/Inc/sh2_hal_spi.h).
2. [sh2_hal_spi.c](Core/Src/sh2_hal_spi.c).
3. [bno085_runner.h](Core/Inc/bno085_runner.h).
4. [bno085_runner.c](Core/Src/bno085_runner.c).

#### 4.6.1 SH2 Set Reorientation Quaternion

The hub composes the re-orientation vector as:

$$q_{\mathrm{out}} = q_{\mathrm{reorient}} \otimes q_{\mathrm{measured}}$$

To make a specific physical pose `P` report as identity (0,0,0,1),
the **inverse of that pose in the sensor/device frame** must be sent, converted
to Q14 and passed to `sh2_setReorientation()`.

---

## 5 BMP390 Barometric Pressure Sensor

24-bit absolute barometric pressure sensor by Bosch Sensortec, designed for
performant altimeter applications. Very small package, I2C and SPI capable.

> Utilized reference documents:
> 1. `BST-BMP390-DS002-07 - BMP390 Datasheet v1.7`

### 5.1 Background

The BMP390 is ideally suited for burst communications over both I2C and SPI. I2C
was chosen due to its simplified wiring and ease of peripheral integration.
Additionally, in most applications, a 9-DOF IMU is likely to be used as the
primary dynamic sensor. _(I also just wanted to not use SPI for everything,
that's kinda boring)_.

### 5.2 Inter-Integrated Circuit (I2C)

As specified by datasheets, I2C Fast Mode is used for the (fast mode standard)
400 kHz clock.

### 5.3 Timer

Similar to the BNO086's timer ([4.4 Timer](#44-timer)), TIM2 is configured to be
used for timing operations (1 µs time base) in the BMP3 drivers.

Since TIM2 is also on APB1, the prescaler calculations are the same as the
BNO086,
see [4.4.1 Timer Prescaler Calculation](#441-timer-prescaler-calculation).

### 5.4 BMP390 Driver

Submodule: [BMP3_SensorAPI](Core/BMP3_SensorAPI).

Source: [github.com/boschsensortec/BMP3_SensorAPI](https://github.com/boschsensortec/BMP3_SensorAPI).

STM32 HAL abstraction and runner functions:

1. [bmp3_hal_i2c.h](Core/Inc/bmp3_hal_i2c.h).
2. [bmp3_hal_i2c.c](Core/Src/bmp3_hal_i2c.c).
3. [bmp390_runner.h](Core/Inc/bmp390_runner.h).
4. [bmp390_runner.c](Core/Src/bmp390_runner.c).

---

## 6 TJA1057BTK CAN Bus Transceiver

CAN transceiver (MCU to 2-wire CAN bus) by NXP. 3V - 5V variant of TJA1057BTK.

> Utilized reference documents:
> 1. `TJA1051 Product data sheet Rev. 8`

### 6.1 Background

The 3V variant is used for convince, just an easy pick. _(I also had experience
with it previously)_.

### 6.2 Controller Area Network (CAN)

#### 6.2.1 Bit Time Calculation

CAN peripherals run on APB1 (80 MHz), the goal is for a 500 kHz CAN bus.

```
Prescaler                    = 16
Time Quanta in Bit Segment 1 = 5     times
Time Quanta in Bit Segment 2 = 4     times
Time Quantum                 = 200.0 ns
```

> Lots of resources and calculators online, example here:
> [http://www.bittiming.can-wiki.info/](http://www.bittiming.can-wiki.info/).

#### 6.2.2 Nested Vectored Interrupt Controller (NVIC)

`CAN1` has the following NVIC configurations:

1. `CAN1 RX0 interrupt`
2. `CAN1 RX1 interrupt`

This enables reception interrupts for interactions based on incoming CAN
messages.

### 6.3 CAN High-Level Driver

1. [can.h](Core/Inc/can.h).
2. [can.c](Core/Src/can.c).

### 6.4 CAN DBC and Low-Level Driver

Low level CAN communication drivers can be found
here: [`momentum_driver`](https://github.com/scalpelspace/momentum_driver).

---

## 7 SAM-M10Q RF Receiver Galileo, GLONASS, GPS

Multi-constellation GPS module by u-blox. GPS, GLONASS, Galileo capable at 18Hz
individually or dual (GPS and GLONASS) at 10 Hz. Supports both UART and I2C.

> Utilized reference documents:
> 1. `SAM-M10Q Data sheet 14-May-2024`
> 2. `SAM-M10Q Integration manual 31-May-2023`
> 3. `M10 firmware 5.10 interface description 11-Jul-2023`

### 7.1 Background

The u-blox SAM module was chosen for its ease of future updates and extremely
easy integration and development.

### 7.2 Universal Synchronous/Asynchronous Receiver/Transmitter (USART)

UART baud rate is set for 9600 bps (default baud rate of u-blox module).

#### 7.2.1 Direct Memory Access (DMA)

DMA is used configured to allow continuous GPS receive in hardware:

`USART2_RX` `DMA1 Stream6`:

- Direction: `Peripheral to Memory`.
- Mode: `Circular`
- Peripheral Increment Address: `Disabled`.
- Memory Increment Address: `Enabled`.
- (Both Peripheral and Memory) Data Width: `Byte`.
- Use FIFO: `Disabled`.

`USART2_TX` `DMA1 Stream7`:

- Direction: `Memory to Peripheral`.
- Mode: `Normal`
- Peripheral Increment Address: `Disabled`.
- Memory Increment Address: `Enabled`.
- (Both Peripheral and Memory) Data Width: `Byte`.
- Use FIFO: `Disabled`.

#### 7.2.2 Nested Vectored Interrupt Controller (NVIC)

USART2 global interrupted is enabled.

### 7.3 SAM-M10Q Driver

STM32 HAL abstraction and runner functions:

1. [ublox_hal_uart.h](Core/Inc/ublox_hal_uart.h).
2. [ublox_hal_uart.c](Core/Src/ublox_hal_uart.c).

---

## 8 WS2812B PWM Addressable RGB LED

Individually addressable RGB LED with an integrated control circuit over a
series single-wire data protocol.

### 8.1 Clocks

APB1: 80 MHz (clock for TIM1 PWM output channels).

All subsequent calculations assume an 80 MHz peripheral clock on the PWM timer
channel.

### 8.2 Pulse Width Modulation (PWM) Timer

A Pulse Width Modulation (PWM) timer is utilized generate data signals to the
WS2812B.

`PA8` → Timer 1 Channel 1 → PWM Generation CH1.

```c
htim1.Instance = TIM1;
htim1.Init.Prescaler = 3-1;
htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
htim1.Init.Period = 25-1;
htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim1.Init.RepetitionCounter = 0;
htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
```

#### 8.2.1 Timer Calculations

Given the PWM equation:

$$f_{PWM} = \frac{f_{TIM}}{ \left( ARR + 1 \right) \times \left( PSC + 1 \right) }$$

- $f_{TIM} = 80 \space \mathrm{MHz}$
    - Defined by the PWM channel's peripheral clock.
- $ARR = 25 - 1$
    - Counter period, aka Auto Reload Register (ARR) of 25 is used to simplify
      the translation of duty cycle percentages.
- $f_{PWM} = 800 \space \mathrm{kHz}$
    - As specified by the WS2812B datasheet, the target data transfer time
      period is 1.25 µs, or $1.25 \times 10 ^{-6} \space \mathrm{s}$.
    - Calculating for required PWM frequency:
        - $f_{PWM} = \frac{1}{1.25 \times 10 ^{-6} \space \mathrm{s}} = 800 \space \mathrm{kHz}$

Thus, the prescaler, $PSC = 3.17 - 1$.

### 8.3 Direct Memory Access (DMA)

Direct Memory Access (DMA) is used to transfer the color data for the WS2812B
LEDs directly from memory to the PWM timer's registers without requiring CPU
overhead.

`TIM1_CH1` `DMA1 Stream2`:

- Direction: `Memory to Peripheral`.
    - Software tells what to send on the output.
- Mode: `Normal`.
    - Send the PWM signal just once (WS2812Bs hold LED settings), not
      continuous.
- Peripheral Increment Address: `Disabled`.
- Peripheral Data Width: `Half Word`.
    - TIM1 is a 16-bit/pulse PWM timer, matching the Half Word (16-bits).
- Memory Increment Address: `Enabled`.
- Memory Data Width: `Byte`.

### 8.4 Nested Vectored Interrupt Controller (NVIC)

Nested Vectored Interrupt Controller (NVIC) is used to efficiently manage the
interrupt generated by the DMA controller upon the completion of a data
transfer. This allows the system to update the PWM signals for the WS2812B LEDs
with minimal CPU overhead, enabling efficient and responsive control of the
LEDs.

On CubeMX, NVIC `DMA1 channel2 global interrupt` is enabled for TIM1.

`HAL_TIM_PWM_PulseFinishedCallback()` is called within the Interrupt Service
Routine (ISR) for end of PWM DMA transmissions.

### 8.5 WS2812B Driver

The WS2812B driver is made of 2 files:

1. [ws2812b_hal_pwm.h](Core/Inc/ws2812b_hal_pwm.h).
2. [ws2812b_hal_pwm.c](Core/Src/ws2812b_hal_pwm.c).

```
ws2812b_init(): Initialize DMA, flags, timers, etc.
↓
ws2812b_set_colour(): Set struct values for (R, G, B) colours.
↓
ws2812b_update(): Initialize DMA buffer and trigger PWM DMA transfer.
↓
ws2812b_callback(): Called in ISR for end of PWM, stop DMA transfer.
```

#### 8.5.1 PWM Duty Cycle Calculations

Duty cycle required for PWM control:

$$D = \frac{PW}{T} \times 100$$

$$Value = \frac{PW}{T} \times \left( ARR + 1 \right)$$
$$Value = \frac{PW}{T} \times 25$$

- $D$ = Duty cycle percentage, required calculation.
- $PW$ = Pulse width (active time), as defined by the datasheet.
- $T$ = Total signal time period, 1.25 µs, as defined by the datasheet.
- $Value$ = Actual digital value to send representing the required duty cycle
  percentage.

| Operation | $PW$   | Margin  | $D$ | Value |
|-----------|--------|---------|-----|-------|
| 0 code    | 0.4 µs | ±150 ns | 32% | 8     |
| 1 code    | 0.8 µs | ±150 ns | 64% | 16    |

#### 8.5.2 Reset Code Time Periods Calculation

The datasheet requires a low signal of > 50 µs. Thus, the minimum number of
full (low) cycles is given by:

$$50 \space \mathrm{\mu s} \div 1.25 \space \mathrm{\mu s} = 40$$

---

## 9 Real Time Clock (RTC)

RTC is enabled and setup for clock and calendar.

### 9.1 RTC Driver

The RTC driver is made of 2 files:

1. [rtc.h](Core/Inc/rtc.h).
2. [rtc.c](Core/Src/rtc.c).

---

## 10 Shared Low-Level Software Features

### 10.1 Callbacks

Callbacks for certain peripherals are shared by the STM32 HAL and are
centralized within the following module:

1. [callbacks.c](Core/Src/callbacks.c).

This approach ensures that callback functions remain atomic and specific to
individual drivers. These atomic functions are then consolidated into a single
function (user implementation), overriding the weak declarations provided by
the STM32 HAL.

---

## 11 Third-Party Licenses

This project uses the following open-source software components:

- **Bosch [BMP3_SensorAPI](https://github.com/boschsensortec/BMP3_SensorAPI)**,
  Bosch Sensortec and contributors.
    - Licensed under the `3-Clause BSD License`.
        - See [
          `LICENSE`](https://github.com/boschsensortec/BMP3_SensorAPI/blob/master/LICENSE).

- **CEVA [sh2](https://github.com/ceva-dsp/sh2)**, CEVA Inc.
    - Licensed under the `Apache License, Version 2.0`.
        - See [
          `NOTICE.txt`](https://github.com/ceva-dsp/sh2/blob/main/NOTICE.txt)
          and code headers, for example in: [
          `sh2.h`](https://github.com/ceva-dsp/sh2/blob/main/sh2.h).

- **STM32Cube HAL**, STMicroelectronics.
    - Licensed under the `3-Clause BSD License`.
        - See [`LICENSE.txt`](Drivers/STM32L4xx_HAL_Driver/LICENSE.txt).

> Bosch, CEVA, and STMicroelectronics are trademarks of their respective
> owners. Use of these names does **not** imply any endorsement by the trademark
> holders.
