# Pin Map (USB v1 + NUCLEO-G431KB)

This document captures the baseline mapping used by `usb_v1` firmware.
Adjust to your exact board/schematic before flashing.

## Board Metadata

- STM32 part number: `STM32G431KBT6` (board: `NUCLEO-G431KB`)
- Package: `LQFP32`
- Baseline date: `2026-04-10`

## Clock / Debug

| Signal | MCU Pin | Notes |
|---|---|---|
| NRST | `NRST` | Reset line |
| BOOT0 | `PB8` | Keep normal flash boot for runtime |
| SWDIO | `PA13` | ST-LINK SWD |
| SWCLK | `PA14` | ST-LINK SWD |

## USB Serial Transport

`usb_v1` expects a serial byte stream from the Pi. Use one of these paths:

1. Native USB CDC (preferred if your project enables USB device stack), or
2. `USART1` through a USB-UART bridge.

Baseline USART pins (fallback path):

| Function | MCU Pin | Peripheral | Notes |
|---|---|---|---|
| USB serial TX | `PA9 (D1)` | `USART1_TX` | To USB-UART RX |
| USB serial RX | `PA10 (D0)` | `USART1_RX` | To USB-UART TX |

## PWM Outputs

| Function | MCU Pin | Timer Channel | Notes |
|---|---|---|---|
| Propulsion PWM | `PA8 (D9)` | `TIM1_CH1` | ESC signal |
| Steering PWM | `PA11 (D10)` | `TIM1_CH4` | Servo signal |

## Inputs (Telemetry Sources)

| Function | MCU Pin | Peripheral | Notes |
|---|---|---|---|
| Wheel speed | `PA0 (A0)` | `TIM2_CH1` input capture | Speed measurement |
| Rear obstacle | `PA3 (A2)` | `ADC/GPIO threshold` | Boolean `rear_obstacle` |
| Rear obstacle (alt) | `PA1 (A1)` | `ADC/GPIO threshold` | Optional alternate source |

## Firmware Binding Checklist

- Serial transport returns complete newline-terminated command lines.
- `Board_UsbReadLine()` and `Board_UsbWriteLine()` are wired to your transport driver.
- PWM timer period generates `50 Hz` servo/ESC base frequency.
- Rear obstacle polarity is confirmed (`1 = obstacle`, `0 = clear`).
- Wheel speed scaling to m/s is documented in board code comments.
