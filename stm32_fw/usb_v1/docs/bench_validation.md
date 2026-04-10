# Bench Validation Checklist

Run with wheels off ground and emergency stop available.

## Firmware-Level Checks

1. Power-on neutral:
- propulsion PWM at stop duty
- steering PWM at center duty

2. Valid command behavior:
- send valid `CMD,...` line
- verify steering/speed outputs update as expected
- with `run_enable=0`, verify STM32 output remains neutral

3. Emergency brake behavior:
- send valid command with `ebrake=1`
- verify immediate neutral output

4. Invalid line behavior:
- malformed CSV or wrong prefix
- verify command is ignored and output does not jump

5. Watchdog behavior:
- stop command stream
- verify neutral output after `250 ms` timeout

6. Telemetry behavior:
- verify `TEL,...` lines include wheel speed and rear obstacle state

## Pi Integration Checks

1. Start:

```bash
ros2 launch covapsy_bringup car_safe.launch.py
```

2. Confirm status:

```bash
ros2 topic echo /bridge_status --once
```

3. Command test:

```bash
ros2 topic pub /race_start std_msgs/msg/Bool "{data: true}" --once
```

4. Watchdog test:
- stop command source
- verify neutral within timeout
- publish `/race_stop` and verify immediate stop

## Pass Criteria

- No unexpected actuator jumps.
- USB link remains stable (`/bridge_status` not stuck in disconnect/timeout).
- Watchdog reliably enforces neutral stop.
- Telemetry topics publish sensible values.
