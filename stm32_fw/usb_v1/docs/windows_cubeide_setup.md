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
2. Open the STM32 project wizard:
   - preferred: `File` -> `New` -> `STM32 Project`
   - if not visible in your UI: `File` -> `New` -> `Other...` -> `Create New STM32 Project`
3. Use the STM32/Cube wizard path (do **not** use a generic C/C++ empty Eclipse project).
4. Select exact MCU from `docs/pinmap.md`.
   - for `NUCLEO-G431KB`, Cube may show `STM32G431KBTx` variants (`KBT3`, `KBT6`, `...TR`); these package variants are fine for this workflow.
5. Project location: `mycar/stm32_fw/usb_v1/cubeide_project`.
6. Keep `Debug` and `Release` build configurations.

## 3. Configure Peripherals (CubeMX view)

Use `docs/pinmap.md` to configure:

- system clock source and frequency
- USB serial transport path:
  - preferred: native USB CDC device (if enabled in your project)
  - fallback: `USART1` (`PA9/PA10`) bridged through USB-UART
- `USART1` async parameters: `115200`, `8N1`, TX/RX enabled
- timer PWM channels for propulsion and steering at `50 Hz`:
  - `TIM1_CH1` on `PA8`
  - `TIM1_CH4` on `PA11`
- wheel speed input capture:
  - `TIM2_CH1` on `PA0` (input capture)
- rear obstacle input:
  - baseline `PA3` (`ADC1_IN4`) thresholded to boolean

### Timer Frequency Sanity Check

The firmware assumes:

- propulsion/steering PWM base frequency = `50 Hz`
- wheel capture timer tick = `1 MHz`

If your timer clock is `16 MHz` (default HSI setup), use:

- `TIM1`: `PSC=15`, `ARR=19999`  -> `50 Hz`
- `TIM2`: `PSC=15` -> `1 MHz` tick

If your timer clock differs, recompute prescaler/period accordingly.

### Board Target / ADC Handle Sanity Check

Before building, confirm generated ADC handle matches `fw_config.h`:

- `NUCLEO-L432KC` target (`STM32L432xx`): `FW_REAR_OBSTACLE_ADC_HANDLE` should be `hadc1`
- `NUCLEO-G431KB` with baseline `PA3` (`ADC1_IN4`): `FW_REAR_OBSTACLE_ADC_HANDLE` should be `hadc1`

If your Cube project generates different handle names/instances, align `fw_config.h` before compiling.

### Code Generation Settings Sanity Check

In `Project Manager`:

- `Toolchain/IDE` must be `STM32CubeIDE` (not `EWARM`)
- enable "Generate peripheral initialization as a pair of `.c/.h` files per peripheral" (so `tim.h`, `usart.h`, `adc.h` are generated)


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

Important config alignment after copy:

- if rear obstacle is on `PA3/ADC1_IN4`, set `FW_REAR_OBSTACLE_ADC_HANDLE` to `hadc1`
- if your project does not configure `I2C1` + OLED, set `FW_OLED_ENABLE` to `0u`

## 5. Build

1. Select `Release` config.
2. Build project.
3. Confirm generated artifact (`.elf` / `.bin`) exists.
4. Copy release files to:
   - `stm32_fw/usb_v1/releases/<version>/`

If build actions are disabled/greyed out in CubeIDE, check:

- project was generated with `Toolchain/IDE = STM32CubeIDE`
- project contains both `.project` and `.cproject`
- project was refreshed/reimported after regeneration

## 6. Next Steps

- Flash using `flashing_swd_dfu.md`.
- Run bench checks in `bench_validation.md`.
- Continue with Pi integration in `pi_integration.md`.
