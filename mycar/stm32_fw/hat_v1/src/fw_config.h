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

/*
 * Baseline STM32 peripheral bindings (NUCLEO-G431KB + HAT v1re2).
 * These symbols are expected from Cube-generated HAL files.
 */
#define FW_SPI_HANDLE hspi3
#define FW_PWM_TIMER_HANDLE htim1
#define FW_PWM_PROP_CHANNEL TIM_CHANNEL_1
#define FW_PWM_STEERING_CHANNEL TIM_CHANNEL_4
#define FW_WHEEL_TIMER_HANDLE htim2
#define FW_WHEEL_TIMER_CHANNEL TIM_CHANNEL_1

/*
 * Wheel speed conversion defaults.
 * Professor baseline: 79 mm per transmission shaft turn, 16 pulses per turn.
 * Timer capture is expected at 1 MHz (1 tick = 1 us).
 */
#define FW_WHEEL_DISTANCE_PER_TURN_MM 79.0f
#define FW_WHEEL_PULSES_PER_TURN 16.0f
#define FW_WHEEL_TIMER_TICK_HZ 1000000.0f
#define FW_WHEEL_SPEED_MAX_ABS_MPS 20.0f

/*
 * Rear obstacle baseline source:
 * - CAPT_IR_D on PA3 (A2)
 * - by default read through ADC and converted to boolean with threshold
 */
#define FW_REAR_OBSTACLE_USE_ADC 1u
#define FW_REAR_OBSTACLE_ADC_HANDLE hadc2
#define FW_REAR_OBSTACLE_ADC_THRESHOLD_RAW 2048u /* 12-bit ADC range: 0..4095 */
#define FW_REAR_OBSTACLE_ACTIVE_HIGH 1u

/* Optional logging (set to 1 if USART is configured and desired). */
#define FW_LOG_USE_UART 0u
#define FW_LOG_UART_HANDLE huart1

#endif /* FW_CONFIG_H */
