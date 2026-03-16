# hat_v1 Firmware v0.2.0

Date: `2026-03-08`
Author: `COVAPSY Team`

## Summary

Rules-compliance hardening for competition control:

- protocol flag `RUN_ENABLE` added (command `FLAGS` bit 0)
- STM32 now forces neutral output when `RUN_ENABLE` is not set
- watchdog behavior retained at `250 ms`

## Protocol

- `FW_PROTOCOL_VERSION`: `1.1.0`
- headers: `0x55 0x55`
- frame size: `6`
- checksum: sum of bytes `[0..4] & 0xFF`
- steering index: `2`
- speed index: `3`
- flags index: `4`
- command flag bit 0 (`0x01`) = `RUN_ENABLE`

## Bridge Compatibility

Expected compatible ROS settings:

- `backend=spi`
- `spi_header_0=85`
- `spi_header_1=85`
- `spi_steering_index=2`
- `spi_speed_index=3`
- `spi_flags_index=4`
- `competition_mode=true`
- `require_start_signal=true`
- `cmd_topic=/cmd_vel_autonomy`

## Flashing

- Primary: ST-LINK SWD
- Fallback: USB DFU

## Validation

- [ ] valid frame with `RUN_ENABLE=1` updates PWM
- [ ] valid frame with `RUN_ENABLE=0` keeps neutral output
- [ ] invalid frame ignored
- [ ] watchdog stop in <= 250 ms
- [ ] rear obstacle bit toggles correctly
- [ ] `/mcu_status` is healthy on Pi safe bringup
