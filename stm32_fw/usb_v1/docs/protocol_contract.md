# USB Protocol Contract (Firmware <-> Pi Bridge)

This contract matches the current ROS bridge implementation in:

- `ros2_ws/src/covapsy_bridge/covapsy_bridge/stm32_bridge_node.py`

## Transport

- Physical link: USB serial (CDC ACM or USB-UART path)
- Encoding: ASCII CSV, newline-terminated (`\n`)
- Pi acts as command source; STM32 enforces low-level safety
- Target cadence: 50 Hz command + 5 Hz LCD status + 50 Hz telemetry

## Command Line (Pi -> STM32)

Format:

`CMD,<seq>,<steer_deg>,<speed_mps>,<run_enable>,<ebrake>`

Fields:

- `seq`: monotonically increasing integer
- `steer_deg`: steering command in degrees
- `speed_mps`: speed command in m/s
- `run_enable`: `1` allows motion, `0` forces neutral
- `ebrake`: `1` requests immediate neutral stop

Firmware behavior:

- clamps steering/speed to firmware limits
- enforces neutral output if `run_enable=0`
- enforces neutral output if `ebrake=1`

## LCD Status Line (Pi -> STM32)

Format:

`LCD,<seq>,<l1>|<l2>|<l3>|<l4>`

Fields:

- `seq`: monotonically increasing display sequence
- `l1..l4`: text lines for a `4x16` character view
  - exactly four segments separated by `|`
  - each segment max `16` printable ASCII characters
  - commas are forbidden in LCD payload

Firmware behavior:

- stores the latest valid LCD frame by sequence
- ignores stale/out-of-order LCD frames
- rendering failure does not affect motion control path

## Telemetry Line (STM32 -> Pi)

Format:

`TEL,<seq>,<wheel_mps>,<rear_obstacle>,<status_code>`

Fields:

- `seq`: last accepted command sequence
- `wheel_mps`: measured wheel speed in m/s
- `rear_obstacle`: `1` obstacle, `0` clear
- `status_code`: firmware status integer
  - `0` normal
  - `1` watchdog forced stop
  - `2` run disabled
  - `3` emergency brake active

## Validation Rules

- command line must contain exactly 6 CSV tokens and start with `CMD`
- LCD line must contain exactly 3 CSV tokens and start with `LCD`
- LCD payload must contain exactly 4 pipe-separated segments (`l1|l2|l3|l4`)
- each LCD segment must be <=16 printable ASCII chars
- telemetry line must contain exactly 5 CSV tokens and start with `TEL`
- malformed lines are ignored; watchdog/race-safety behavior still applies
- command loss beyond watchdog timeout must force neutral output
