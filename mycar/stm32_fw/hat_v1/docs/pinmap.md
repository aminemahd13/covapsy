# Pin Map (Professor Baseline: HAT v1re2 + NUCLEO-G431KB)

This mapping is extracted from:

- `Covapsy-prof/Hardware/cartes_electroniques/Hat_CoVASPSy_v1re2.sch`
- `Covapsy-prof/Software/MiseEnOeuvre_Entrees_Sorties_STM32.pdf`
- `Covapsy-prof/Software/MiseEnOeuvre_FourcheOptique_STM32.pdf`
- ST schematic `MB1430-G431KBT6-A02` (NUCLEO-G431KB)

Board and schematic metadata:

- HAT revision: `v1re2`
- STM32 part number: `STM32G431KBT6` (board: `NUCLEO-G431KB`)
- Package: `LQFP32`
- Schematic source file name: `Hat_CoVASPSy_v1re2.sch`
- Date extracted: `2026-03-08`
- Extracted by: `Covapsy firmware audit`

## Clock / Reset / Boot

| Signal | MCU Pin | Net Name (schematic) | Notes |
|---|---|---|---|
| HSE_IN / crystal | `PF0` | `OSC_IN` | Optional external HSE path on Nucleo; default firmware uses HSI |
| HSE_OUT / crystal | `PF1` | `OSC_OUT` | Optional external HSE path on Nucleo |
| NRST | `NRST` | `RESET` | Routed to Nano reset pin and ST-LINK reset |
| BOOT0 | `PB8` | `BOOT0` | `PB8-BOOT0` on Nucleo; keep normal boot from flash for runtime |
| SWDIO | `PA13` | `T_SWDIO` | ST-LINK SWD |
| SWCLK | `PA14` | `T_SWCLK` | ST-LINK SWD |

## SPI (Pi Master <-> STM32 Slave)

| Function | MCU Pin | Alt Function | Net Name | Notes |
|---|---|---|---|---|
| SPI_NSS (CS) | `Not wired` | `N/A` | `SS` (RPi pin 24 only) | HAT does not route CS to STM32; use slave mode with software NSS |
| SPI_SCK | `PB3 (D13)` | `AF6 SPI3_SCK` | `SCK` | Mode 0 compatibility |
| SPI_MISO | `PB4 (D12)` | `AF6 SPI3_MISO` | `MISO` | STM32 -> Pi |
| SPI_MOSI | `PB5 (D11)` | `AF6 SPI3_MOSI` | `MOSI` | Pi -> STM32 |

## PWM Outputs

| Function | MCU Pin | Timer Channel | Net Name | Notes |
|---|---|---|---|---|
| Propulsion PWM | `PA8 (D9)` | `TIM1_CH1` | `PROPULSION_MCU` | ESC signal |
| Steering PWM | `PA11 (D10)` | `TIM1_CH4` | `SERVO_DIR_MCU` | Servo signal |

## Inputs (Telemetry Sources)

| Function | MCU Pin | Peripheral | Net Name | Notes |
|---|---|---|---|---|
| Wheel speed | `PA0 (A0)` | `TIM2_CH1` input capture | `FOURCHE` | Recommended for speed measurement |
| Rear obstacle | `PA3 (A2)` | `ADC/GPIO threshold` | `CAPT_IR_D` | Default candidate for boolean `rear_obstacle` |
| Rear obstacle (alt) | `PA1 (A1)` | `ADC/GPIO threshold` | `CAPT_IR_G` | Optional alternate sensor source |

## Optional Interfaces

| Function | MCU Pin | Peripheral | Net Name | Notes |
|---|---|---|---|---|
| UART TX | `PA9 (D1)` | `USART1_TX` | `TX_MCU` | Optional debug / interface UART |
| UART RX | `PA10 (D0)` | `USART1_RX` | `RX_MCU` | Optional debug / interface UART |
| TFT control lines | `N/A on hat_v1` | `N/A` | `N/A` | Not part of the baseline HAT mapping |

## Firmware Binding Checklist

- SPI uses `SPI3` on `PB3/PB4/PB5` in slave mode, SPI mode `0`, with software NSS.
- `Board_SpiReadFrame()` returns exactly one 6-byte frame per transaction cycle.
- PWM timer period generates `50 Hz` servo/ESC base frequency.
- Rear obstacle polarity is confirmed in board code (`1 = obstacle`, `0 = clear`).
- Wheel speed scaling to m/s is documented in board code comments.
