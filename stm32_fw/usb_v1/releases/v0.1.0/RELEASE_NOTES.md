# usb_v1 Firmware v0.1.0 (Legacy SPI Baseline)

Date: `2026-03-08`
Author: `COVAPSY Team`

## Summary

Historical baseline before USB migration:

- 6-byte SPI command parser
- PWM command mapping for propulsion and steering
- 250 ms watchdog to neutral outputs
- telemetry response frame with wheel speed + rear obstacle flag

## Protocol

- `FW_PROTOCOL_VERSION`: `1.0.0`
- transport: SPI binary frame
- command flag bit 0 (`RUN_ENABLE`) not yet introduced

## Migration Note

This release is retained for traceability only. Active development now targets USB CSV protocol in `FW_PROTOCOL_VERSION >= 2.0.0`.
