# STM32 Firmware Workspace (`usb_v1`)

This folder is the STM32-side firmware workspace for USB-connected operation with the Raspberry Pi 5.

## Scope

- USB serial command protocol (CSV text lines, newline-terminated)
- Steering + propulsion PWM output mapping
- 250 ms command watchdog forcing neutral output
- Telemetry line output (`wheel_speed`, `rear_obstacle`, `status_code`)
- Windows STM32CubeIDE build and SWD/DFU flashing runbooks

## Folder Layout

- `cubeide_project/`: STM32CubeIDE project location (generated files live here)
- `src/`: protocol and control logic source files to copy/import into CubeIDE project
- `docs/`: implementation and bringup docs
- `releases/`: built artifact inventory and release notes

## Required First Step

Before compiling on hardware, complete and verify pin/transport mapping in:

- `docs/pinmap.md`

## Protocol Compatibility Baseline

The defaults in this firmware workspace match:

- `ros2_ws/src/covapsy_bridge/covapsy_bridge/stm32_bridge_node.py`
- `ros2_ws/src/covapsy_bringup/config/learn_real.yaml`
- `ros2_ws/src/covapsy_bringup/config/race_real.yaml`

Current defaults:

- line format (Pi -> STM32): `CMD,<seq>,<steer_deg>,<speed_mps>,<run_enable>,<ebrake>`
- line format (STM32 -> Pi): `TEL,<seq>,<wheel_mps>,<rear_obstacle>,<status_code>`
- serial baud on Pi side: `115200`
- serial device on Pi side: `/dev/stm32_mcu`

## Quick Build Sequence (Windows)

1. Complete `docs/pinmap.md` for your exact board wiring.
2. Follow `docs/windows_cubeide_setup.md`.
3. Import `src/` files into CubeIDE project under `cubeide_project/`.
4. Flash via `docs/flashing_swd_dfu.md` (SWD first, DFU fallback).
5. Validate with `docs/bench_validation.md`.
6. Integrate on Pi with `docs/pi_integration.md`.
