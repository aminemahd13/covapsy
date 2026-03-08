# STM32 Firmware Workspace (`hat_v1`)

This folder is the STM32 side implementation for the custom HAT, aligned with the
existing Raspberry Pi ROS bridge (`spi` backend).

## Scope

- SPI slave drive command protocol (6-byte frame, header `0x55 0x55`, software NSS)
- Steering + propulsion PWM output mapping
- 250 ms command watchdog to force neutral output
- Telemetry response frame (`wheel_speed`, `rear_obstacle`)
- Windows STM32CubeIDE build and SWD/DFU flashing runbooks

## Folder Layout

- `cubeide_project/`: STM32CubeIDE project location (generated project files live here)
- `src/`: protocol and control logic source files to copy/import into CubeIDE project
- `docs/`: implementation and bringup docs
- `releases/`: built artifact inventory and release notes

## Required First Step

Before compiling on hardware, complete the pin mapping in:

- `docs/pinmap.md`

The mapping must come from the professor schematic for the exact HAT revision.

## Protocol Compatibility Baseline

The defaults in this firmware workspace match:

- `ros2_ws/src/covapsy_bringup/config/backend_params.yaml`
- `ros2_ws/src/covapsy_bridge/covapsy_bridge/stm32_bridge_node.py`

Current defaults:

- frame size: `6`
- headers: `0x55`, `0x55`
- steering byte index: `2`
- speed byte index: `3`
- flags byte index: `4`
- checksum byte index: `5`
- SPI mode: `0`
- SPI speed on Pi side: `1 MHz`

## Quick Build Sequence (Windows)

1. Complete `docs/pinmap.md`.
2. Follow `docs/windows_cubeide_setup.md`.
3. Import `src/` files into CubeIDE project under `cubeide_project/`.
4. Flash via `docs/flashing_swd_dfu.md` (SWD first, DFU fallback).
5. Validate with `docs/bench_validation.md`.
6. Integrate on Pi with `docs/pi_integration.md`.
