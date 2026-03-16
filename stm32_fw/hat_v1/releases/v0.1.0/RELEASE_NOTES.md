# hat_v1 Firmware v0.1.0

Date: `2026-03-08`
Author: `COVAPSY Team`

## Summary

Initial SPI minimal firmware baseline:

- 6-byte SPI command parser
- PWM command mapping for propulsion and steering
- 250 ms watchdog to neutral outputs
- telemetry response frame with wheel speed + rear obstacle flag

## Protocol

- `FW_PROTOCOL_VERSION`: `1.0.0`
- headers: `0x55 0x55`
- frame size: `6`
- checksum: sum of bytes `[0..4] & 0xFF`
- steering index: `2`
- speed index: `3`
- flags index: `4`

## Bridge Compatibility

Expected compatible ROS settings:

- `backend=spi`
- `spi_header_0=85`
- `spi_header_1=85`
- `spi_steering_index=2`
- `spi_speed_index=3`
- `spi_flags_index=4`
- `spi_mode=0`

## Flashing

- Primary: ST-LINK SWD
- Fallback: USB DFU

## Validation

- [ ] valid frame updates PWM
- [ ] invalid frame ignored
- [ ] watchdog stop in <= 250 ms
- [ ] rear obstacle bit toggles correctly
- [ ] `/mcu_status` is healthy on Pi safe bringup
