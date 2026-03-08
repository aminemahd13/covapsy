# HAT Jumpers by Backend

Sources used:
- `Covapsy-prof/Hardware/Connecteurs_Cavaliers_CarteHat.pdf`
- `Covapsy-prof/Software/MiseEnOeuvre_Communication_SPI_STM32_RPI.pdf`
- `Covapsy-prof/Hardware/cartes_electroniques/Hat_CoVASPSy_v1re2_Schema.pdf`

## What the Professor Docs Specify
From `Connecteurs_Cavaliers_CarteHat.pdf`:
- Propulsion PWM source jumper:
  - left: signal from microcontroller (STM32)
  - right: signal from nano-computer (Raspberry Pi / Jetson)
- Steering PWM source jumper:
  - left: signal from microcontroller (STM32)
  - right: signal from nano-computer
- UART signal source on interface ribbon:
  - left: signals from microcontroller
  - right: signals from nano-computer
- For Pi-controlled steering, use hardware PWM outputs.

## Backend Mapping
### `spi` backend (recommended default)
- Control path: Pi SPI master -> STM32 SPI slave.
- PWM generation: STM32.
- Jumper position:
  - propulsion PWM source -> STM32 side (left)
  - steering PWM source -> STM32 side (left)

### `uart` backend
- Control path: Pi UART -> STM32 UART.
- PWM generation: STM32.
- Jumper position:
  - propulsion PWM source -> STM32 side (left)
  - steering PWM source -> STM32 side (left)
- Ensure UART routing and connector wiring matches your board revision.

### `pi_pwm` backend
- Control path: Pi directly drives ESC/servo.
- PWM generation: Raspberry Pi hardware PWM.
- Jumper position:
  - propulsion PWM source -> nano-computer side (right)
  - steering PWM source -> nano-computer side (right)

## SPI Framing Note
From professor SPI guide:
- 6-byte transfer cycles.
- tutorial examples use `0x55 0x55` frame prefix and `spi.mode = 0`, `1 MHz`.
- Slave-select (`SS`) is not wired to STM32 on the HAT baseline, so STM32 side uses software NSS.

`backend_params.yaml` exposes these fields:
- `spi_header_0`, `spi_header_1`
- `spi_steering_index`, `spi_speed_index`, `spi_flags_index`

Adjust them if your STM32 firmware expects a different map.

## Power-On Safety Checklist
1. Verify jumper positions physically match selected backend.
2. Verify no mixed source on propulsion/steering unless intentionally testing.
3. Start with wheels off ground after any jumper change.
4. Confirm `/mcu_status` reports correct backend and no init errors.
