# STM32 Firmware Setup

This project now includes a dedicated STM32 firmware workspace for the custom HAT:

- `mycar/stm32_fw/hat_v1`

Use this as the source of truth for:

- firmware protocol and watchdog behavior for the SPI backend
- Windows STM32CubeIDE setup
- ST-LINK SWD and USB DFU flashing
- bench validation and Pi/ROS integration steps

Start here:

- [Firmware Workspace README](../stm32_fw/hat_v1/README.md)
- [Pin Map Template](../stm32_fw/hat_v1/docs/pinmap.md)
- [Protocol Contract](../stm32_fw/hat_v1/docs/protocol_contract.md)
- [Windows CubeIDE Setup](../stm32_fw/hat_v1/docs/windows_cubeide_setup.md)
- [Flashing (SWD + DFU)](../stm32_fw/hat_v1/docs/flashing_swd_dfu.md)
- [Pi Integration](../stm32_fw/hat_v1/docs/pi_integration.md)
- [Bench Validation](../stm32_fw/hat_v1/docs/bench_validation.md)
