# Firmware Source Modules

These files implement protocol and control logic independent from STM32 HAL details.

## Files

- `fw_config.h`: protocol and calibration constants
- `fw_protocol.h/.c`: CSV command parsing and telemetry line formatting
- `control_mapping.h/.c`: steering/speed to PWM mapping
- `fw_app.h/.c`: watchdog and loop-level application state
- `board_if.h`: hardware abstraction interface
- `board_port_template.c`: baseline HAL port (UART-backed USB + TIM1 PWM + TIM2 wheel speed + rear obstacle); adjust for your Cube project
- `main.c`: reference main loop showing integration order

## Porting Flow

1. Configure pins/peripherals in CubeMX from `../docs/pinmap.md`.
2. Copy these files into `Core/Inc` and `Core/Src`.
3. Keep `board_port_template.c` as baseline, or rename and adapt handle/channel macros in `fw_config.h` for your project naming.
4. Merge `main.c` logic into generated `main.c` loop.
