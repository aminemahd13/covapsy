# usb_v1 Firmware v2.0.0

Date: `2026-04-10`
Author: `COVAPSY Team`

## Summary

USB migration release:

- SPI transport removed
- newline CSV command parser: `CMD,<seq>,<steer_deg>,<speed_mps>,<run_enable>,<ebrake>`
- newline CSV telemetry output: `TEL,<seq>,<wheel_mps>,<rear_obstacle>,<status_code>`
- watchdog and `RUN_ENABLE` neutral-stop safety retained

## Protocol

- `FW_PROTOCOL_VERSION`: `2.0.0`
- default serial transport target: USB serial @ `115200`

## Bridge Compatibility

Expected ROS bridge behavior:

- `serial_device=/dev/stm32_mcu`
- `serial_baud=115200`
- `telemetry_timeout_s=0.25`

## Validation

- [ ] valid command lines update PWM
- [ ] `run_enable=0` forces neutral output
- [ ] `ebrake=1` forces neutral output
- [ ] malformed lines are ignored safely
- [ ] watchdog stop in <= 250 ms
- [ ] telemetry lines include wheel speed + rear obstacle + status code
