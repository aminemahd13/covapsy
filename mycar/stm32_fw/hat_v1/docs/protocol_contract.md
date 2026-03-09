# SPI Protocol Contract (Firmware <-> Pi Bridge)

This contract is implemented to match the ROS bridge defaults:

- `ros2_ws/src/covapsy_bringup/config/backend_params.yaml`
- `ros2_ws/src/covapsy_bridge/covapsy_bridge/command_mapping.py`

## Transport

- SPI mode: `0`
- Frame length: `6` bytes
- Pi acts as master, STM32 acts as slave
- Full duplex: each transfer sends command (Pi->STM32) and receives telemetry (STM32->Pi)
- Baseline HAT wiring has no STM32 NSS/CS line; configure STM32 SPI in software NSS mode.

## Command Frame (Pi -> STM32)

Layout:

`[H0, H1, STEER, SPEED, FLAGS, CHECKSUM]`

- `H0`: `0x55`
- `H1`: `0x55`
- `STEER`: uint8 encoded steering
- `SPEED`: uint8 encoded speed
- `FLAGS`: command flags bitmap
  - bit 0 (`0x01`) = `RUN_ENABLE`
    - `1` -> STM32 may apply steering/speed command
    - `0` -> STM32 forces neutral output
- `CHECKSUM`: `(H0 + H1 + STEER + SPEED + FLAGS) & 0xFF`

Decode ranges:

- Steering: `[-18.0, +18.0]` deg
- Speed: `[-4.0, +2.0]` m/s

Decode formula:

1. `norm = byte / 255.0`
2. `value = min_v + norm * (max_v - min_v)`

## Telemetry Frame (STM32 -> Pi)

Layout:

`[H0, H1, STEER, SPEED, FLAGS, CHECKSUM]`

Usage for current bridge parser:

- `SPEED` (index `3`) carries wheel speed encoded over same speed range (`-4..+2` m/s).
- `FLAGS` (index `4`) uses bit 0 for `rear_obstacle`.
  - `1` -> obstacle detected
  - `0` -> no obstacle

`STEER` can be set to commanded steering or reserved telemetry for future extension.

## Validation Rules

STM32 must reject command frame if any check fails:

- header mismatch
- checksum mismatch
- frame length mismatch (not `6`)

On invalid frame:

- keep last valid command
- do not introduce actuator jump
- still enforce watchdog timeout

Competition guard:

- Even with a valid frame, STM32 forces neutral output when `RUN_ENABLE` is not set.
- This provides protocol-level enforcement for race start/stop gating from the ROS bridge.
