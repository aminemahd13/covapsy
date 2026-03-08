#ifndef FW_CONFIG_H
#define FW_CONFIG_H

/* Protocol version must be copied into release notes for compatibility tracking. */
#define FW_PROTOCOL_VERSION "1.0.0"

#define FW_PROTOCOL_FRAME_SIZE 6u
#define FW_PROTOCOL_HEADER0 0x55u
#define FW_PROTOCOL_HEADER1 0x55u
#define FW_PROTOCOL_STEERING_INDEX 2u
#define FW_PROTOCOL_SPEED_INDEX 3u
#define FW_PROTOCOL_FLAGS_INDEX 4u
#define FW_PROTOCOL_CHECKSUM_INDEX 5u

#define FW_MAX_STEERING_DEG 18.0f
#define FW_MAX_SPEED_FWD 2.0f
#define FW_MAX_SPEED_REV -4.0f

#define FW_WATCHDOG_TIMEOUT_MS 250u
#define FW_CONTROL_LOOP_PERIOD_MS 1u

/* Baseline PWM calibration values aligned with professor STM32 calibration guide. */
#define FW_PWM_PROP_STOP 7.5f
#define FW_PWM_PROP_DEADBAND 0.56f
#define FW_PWM_PROP_DELTA_MAX 1.0f
#define FW_PWM_SPEED_HARD 8.0f
#define FW_PWM_DIRECTION_SIGN -1.0f

#define FW_PWM_STEERING_MIN 5.5f
#define FW_PWM_STEERING_MAX 9.3f
#define FW_PWM_STEERING_CENTER 7.4f

#endif /* FW_CONFIG_H */
