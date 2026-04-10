# Firmware Versioning

Firmware versioning is tracked in two places:

- source constant: `FW_PROTOCOL_VERSION` in `src/fw_config.h`
- release note entry under `releases/<version>/RELEASE_NOTES.md`

## Rule

If protocol-related constants or wire formats change, increment protocol version and document
bridge compatibility impact.

Protocol-related items include:

- command line shape (`CMD,...` fields)
- telemetry line shape (`TEL,...` fields)
- run/ebrake semantics
- steering/speed ranges
- watchdog behavior that affects bridge interpretation

## Current

- `FW_PROTOCOL_VERSION = "2.0.0"`
- Bridge compatibility target:
  - command: `CMD,<seq>,<steer_deg>,<speed_mps>,<run_enable>,<ebrake>`
  - telemetry: `TEL,<seq>,<wheel_mps>,<rear_obstacle>,<status_code>`
  - serial default: `115200` baud
