# Firmware Versioning

Firmware versioning is tracked in two places:

- source constant: `FW_PROTOCOL_VERSION` in `src/fw_config.h`
- release note entry under `releases/<version>/RELEASE_NOTES.md`

## Rule

If protocol-related constants change, increment protocol version and document
bridge compatibility impact.

Protocol-related constants include:

- frame size
- headers
- byte indexes
- checksum policy
- speed/steering encoding ranges

## Current

- `FW_PROTOCOL_VERSION = "1.1.0"`
- Bridge compatibility target:
  - `spi_header_0=85`
  - `spi_header_1=85`
  - `spi_steering_index=2`
  - `spi_speed_index=3`
  - `spi_flags_index=4`
  - command flags bit 0 (`RUN_ENABLE`) must be asserted for motion
