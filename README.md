# momentum

![arm_gcc_build](https://github.com/danielljeon/momentum/actions/workflows/arm_gcc_build.yaml/badge.svg)

Microcontroller firmware
for [`momentum_pcb`](https://github.com/danielljeon/momentum_pcb).

- SPI sensor hub compatible with Arduino Uno-style microcontroller boards.

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
<!-- TOC -->

</details>

---

## 1 Overview

### 1.1 Bill of Materials (BOM)

| Manufacturer Part Number | Manufacturer            | Description                       | Quantity | Notes                 |
|--------------------------|-------------------------|-----------------------------------|---------:|-----------------------|
| STM32L432KC              | STMicroelectronics      | 32-bit MCU                        |        1 |                       |
| CP2102N-A02-GQFN24R      | Silicon Labs            | USB 2.0 to UART Interface         |        1 |                       |
| BNO085                   | CEVA Technologies, Inc. | 9-DOF IMU                         |        1 |                       |
| BMP390                   | Bosch Sensortec         | Barometric Pressure Sensor        |        1 |                       |
| TJA1051T/3               | NXP USA Inc.            | CAN Bus Transceiver               |        1 | WIP, under evaluation |
| SAM-M10Q                 | u-blox                  | RF Receiver Galileo, GLONASS, GPS |        1 |                       |
| WS2812B                  | (Various)               | PWM Addressable RGB LED           |        1 |                       |

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

| STM32F446RE | Peripheral              | Config                | Connection                       | Notes                                 |
|-------------|-------------------------|-----------------------|----------------------------------|---------------------------------------|
| PA14        | `SYS_JTCK-SWCLK`        |                       | TC2050 SWD Pin 4: `SWCLK`        |                                       |
| PA13        | `SYS_JTMS-SWDIO`        |                       | TC2050 SWD Pin 2: `SWDIO`        |                                       |
|             | `TIM2_CH1`              | PWM no output         |                                  | BMP390 BMP3 driver timer.             |
|             | `TIM2_CH2`              | PWM no output         |                                  | BNO085 SH2 driver timer.              |
| PA5         | `SPI1_SCK`              |                       | BNO085 Pin 19: `H_SCL/SCK/RX`    |                                       |
| PA4         | `GPIO_Output` (SPI2 CS) | Pull-up, set high     | BNO085 Pin 18: `H_CSN`           |                                       |
| PA6         | `SPI1_MISO`             |                       | BNO085 Pin 20: `H_SDA/H_MISO/TX` |                                       |
| PA7         | `SPI1_MOSI`             |                       | BNO085 Pin 17: `SA0/H_MOSI`      |                                       |
| PB0         | `GPIO_EXTI0`            | Pull-up, falling edge | BNO085 Pin 14: `H_INTN`          |                                       |
| PB1         | `GPIO_Output`           |                       | BNO085 Pin 6: `PS0/Wake`         | Pull low to trigger wake.             |
|             |                         | Hardware pull-up      | BNO085 Pin 5: `PS1`              |                                       |
| PA1         | `GPIO_Output`           |                       | BNO085 Pin 11: `NRST`            | Pull low to reset.                    |
| PB6         | `I2C1_SCL`              |                       | BMP390 Pin 2: `SCK`              |                                       |
| PB7         | `I2C1_SDA`              |                       | BMP390 Pin 4: `SDI`              |                                       |
| PA3         | `USART2_RX`             | 115200 bps            | SAM-M10Q Pin 13: `TXD`           |                                       |
| PA2         | `USART2_TX`             | 115200 bps            | SAM-M10Q Pin 14: `RXD`           |                                       |
| PC15        | `GPIO_Output`           |                       | SAM-M10Q Pin 18: `RESET_N`       | Pull low to reset (>= 1 ms).          |
| PA10        | `USART1_RX`             | 115200 bps            | CP2102N-A02-GQFN24R Pin 20: TXD  |                                       |
| PA9         | `USART1_TX`             | 115200 bps            | CP2102N-A02-GQFN24R Pin 21: RXD  |                                       |
| PA11        | `CAN1_RX`               |                       | TJA1051T/3 Pin 1: `TXD`          |                                       |
| PA12        | `CAN1_TX`               |                       | TJA1051T/3 Pin 4: `RXD`          |                                       |
| PA8         | `TIM1_CH1`              | PWM Generation CH1    | WS2812B Pin: `DIN`               | DIN pin number depends on IC variant. |

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
