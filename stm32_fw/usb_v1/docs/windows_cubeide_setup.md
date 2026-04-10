# Windows STM32CubeIDE Setup

This runbook creates a project in `stm32_fw/usb_v1/cubeide_project` and imports the firmware logic from `src/`.

## 1. Install Tooling

Install on Windows:

- STM32CubeIDE (latest stable)
- STM32CubeProgrammer
- ST-LINK USB driver

Verify ST-LINK connection with STM32CubeProgrammer before opening CubeIDE.

## 2. Create CubeIDE Project

1. Open STM32CubeIDE.
2. `File` -> `New` -> `STM32 Project`.
3. Select exact MCU from `docs/pinmap.md`.
4. Project location: `mycar/stm32_fw/usb_v1/cubeide_project`.
5. Build configuration: keep `Debug` and `Release`.

## 3. Configure Peripherals (CubeMX view)

Use `docs/pinmap.md` to configure:

- system clock source and frequency
- USB serial transport path:
  - preferred: native USB CDC device (if enabled in your project)
  - fallback: `USART1` (`PA9/PA10`) bridged through USB-UART
- timer PWM channels for propulsion and steering at `50 Hz`
- GPIO/EXTI or ADC for rear obstacle input
- timer input-capture or equivalent for wheel speed source

## 4. Import Firmware Logic

Copy files from `src/` into your project `Core/Src` and `Core/Inc`:

- `fw_config.h`
- `fw_protocol.h`, `fw_protocol.c`
- `control_mapping.h`, `control_mapping.c`
- `fw_app.h`, `fw_app.c`
- `board_if.h`
- use `board_port_template.c` as baseline board port
- use `main.c` as integration reference

Then wire board functions to HAL generated code.

## 5. Build

1. Select `Release` config.
2. Build project.
3. Confirm generated artifact (`.elf` / `.bin`) exists.
4. Copy release files to:
   - `stm32_fw/usb_v1/releases/<version>/`

## 6. Next Steps

- Flash using `flashing_swd_dfu.md`.
- Run bench checks in `bench_validation.md`.
- Continue with Pi integration in `pi_integration.md`.
