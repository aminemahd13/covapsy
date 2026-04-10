# usb_v1 Firmware v2.1.0

- Date: 2026-04-10
- Protocol version: `2.1.0`

## Summary

- Added USB LCD relay support: `LCD,<seq>,<l1>|<l2>|<l3>|<l4>`
- Added STM32-side OLED rendering path (SSD1306/SH1106 probe at `0x3C` / `0x3D`)
- Preserved control/telemetry wire format for:
  - `CMD,<seq>,<steer_deg>,<speed_mps>,<run_enable>,<ebrake>`
  - `TEL,<seq>,<wheel_mps>,<rear_obstacle>,<status_code>`

## Compatibility

- Existing `CMD`/`TEL` bridge behavior remains compatible.
- New `LCD` line is optional; firmware ignores malformed LCD lines.
- OLED absence/failure is non-fatal and does not block motion control.

## Validation

- Host parser smoke test passed:
  - `test_fw_protocol_lcd.c`
  - command parser backward compatibility + LCD parser + sequence ordering checks
