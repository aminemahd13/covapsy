# Firmware Source Modules

These files implement protocol and control logic independent from STM32 HAL details.

## Files

- `fw_config.h`: protocol and calibration constants
- `fw_protocol.h/.c`: frame decode/encode and checksum
- `control_mapping.h/.c`: steering/speed to PWM mapping
- `fw_app.h/.c`: watchdog and loop-level application state
- `board_if.h`: hardware abstraction interface
- `board_port_template.c`: template implementation to port with HAL code
- `main.c`: reference main loop showing integration order

## Porting Flow

1. Configure pins/peripherals in CubeMX from `../docs/pinmap.md`.
2. Copy these files into `Core/Inc` and `Core/Src`.
3. Rename `board_port_template.c` to a project-specific file and implement HAL calls.
4. Merge `main.c` logic into generated `main.c` loop.
