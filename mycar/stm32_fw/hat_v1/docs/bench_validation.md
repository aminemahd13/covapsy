# Bench Validation Checklist

Run with wheels off ground and emergency stop available.

## Firmware-Level Checks

1. Power-on neutral:
- propulsion PWM at stop duty
- steering PWM at center duty

2. Valid frame behavior:
- send frame with correct header/checksum
- verify steering/speed outputs update as expected

3. Invalid frame behavior:
- wrong header or checksum
- verify command is ignored and output does not jump

4. Watchdog behavior:
- stop SPI command stream
- verify neutral output after `250 ms` timeout

5. Telemetry frame behavior:
- wheel speed encodes to speed byte
- rear obstacle maps to flags bit 0

## Pi Integration Checks

1. Start:

```bash
ros2 launch covapsy_bringup car_safe.launch.py backend:=spi
```

2. Confirm status:

```bash
ros2 topic echo /mcu_status --once
```

3. Command test:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.1}}" -r 10
```

4. Watchdog test:
- stop publisher
- verify neutral within timeout

## Pass Criteria

- No unexpected actuator jumps.
- SPI status is healthy in `/mcu_status`.
- Watchdog reliably enforces neutral stop.
- Telemetry topics publish sensible values.
