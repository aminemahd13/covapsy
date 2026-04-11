# CubeIDE Projects

## Active Build/Flash Target

- `covapsy_usb1` is the only valid STM32CubeIDE project for `usb_v1`.
- Build and flash exclusively from:
  - `stm32_fw/usb_v1/cubeide_project/covapsy_usb1`

## Placeholder Project (Do Not Flash)

- `covapsy_usb_v1` is retained only as a historical placeholder snapshot.
- Do not build or flash binaries from `covapsy_usb_v1`.

## Source of Truth

- Keep `stm32_fw/usb_v1/src/` as the canonical firmware logic source.
- Sync updates into `covapsy_usb1/Core/*` when required by CubeIDE workflow.

## Ignore Build Outputs

- `Debug/`
- `Release/`
- `.settings/`
