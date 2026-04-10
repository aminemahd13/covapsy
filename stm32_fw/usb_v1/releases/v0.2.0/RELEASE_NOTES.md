# usb_v1 Firmware v0.2.0 (Legacy SPI Baseline)

Date: `2026-03-08`
Author: `COVAPSY Team`

## Summary

Historical SPI hardening release:

- `RUN_ENABLE` flag enforcement
- neutral output when `RUN_ENABLE` is not set
- watchdog behavior retained at `250 ms`

## Protocol

- `FW_PROTOCOL_VERSION`: `1.1.0`
- transport: SPI binary frame with checksum

## Migration Note

This release is retained for traceability only. Active development now targets USB CSV protocol in `FW_PROTOCOL_VERSION >= 2.0.0`.
