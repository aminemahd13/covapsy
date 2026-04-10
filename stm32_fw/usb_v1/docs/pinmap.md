# Pin Map (USB v1 + NUCLEO-L432KC / NUCLEO-G431KB)

This document captures the schematic-validated mapping used by `usb_v1` firmware.
Adjust only if your hardware differs from `schematics/`.

## Board Metadata

- Supported boards:
  - `NUCLEO-L432KC` (`STM32L432KCU6`)
  - `NUCLEO-G431KB` (`STM32G431KBTx`, e.g. `KBT3` / `KBT6`)
- Package: `LQFP32`
- Baseline date: `2026-04-10`

## Schematic to MCU Header Mapping

The nets below come from:

- `schematics/Hat_CoVASPSy_v1re2_Schema.pdf`
- `schematics/Interface_CoVASPSy_v1re2_Schema.pdf`

| Schematic Net | Hat Header Signal | Arduino Header Pin | MCU Pin (L432/G431) | Peripheral Role | Firmware Binding |
|---|---|---|---|---|---|
| `TX_MCU` | UART TX to bridge | `D1` | `PA9` | `USART1_TX` | `FW_USB_UART_HANDLE` |
| `RX_MCU` | UART RX from bridge | `D0` | `PA10` | `USART1_RX` | `FW_USB_UART_HANDLE` |
| `PROPULSION_MCU` | ESC PWM | `D9` | `PA8` | `TIM1_CH1` | `FW_PWM_PROP_CHANNEL` |
| `SERVO_DIR_MCU` | Steering PWM | `D10` | `PA11` | `TIM1_CH4` | `FW_PWM_STEERING_CHANNEL` |
| `FOURCHE` | Wheel speed input | `A0` | `PA0` | `TIM2_CH1` input capture | `FW_WHEEL_TIMER_CHANNEL` |
| `CAPT_IR_D` | Rear obstacle input | `A2` | `PA3` | `ADC/GPIO threshold` | `FW_REAR_OBSTACLE_ADC_HANDLE` |

## Required Jumper Positions (STM32 Control Mode)

On `Hat_CoVASPSy_v1re2`, set:

- `J_PROP`: `1-2` (`PROPULSION_MCU -> PWM_PROPULSION`)
- `J_DIR`: `1-2` (`SERVO_DIR_MCU -> PWM_DIRECTION`)
- `J_TX`: `2-3` (`TX_MCU -> TX_DIR`)
- `J_RX`: `2-3` (`RX_MCU -> RX_DIR`)

## Important Electrical Note

- `PROPULSION_MCU` is routed to `D9` and also tied to `D8` through `R78` on the Hat schematic.
- Keep `D8` un-driven (input/high-Z) in firmware to avoid output contention on the propulsion net.

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
