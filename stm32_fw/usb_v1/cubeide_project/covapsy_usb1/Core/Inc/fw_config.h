#ifndef FW_CONFIG_H
#define FW_CONFIG_H

/* Protocol version must be copied into release notes for compatibility tracking. */
#define FW_PROTOCOL_VERSION "2.1.0"

#define FW_PROTOCOL_LINE_MAX 96u

#define FW_LCD_LINE_COUNT 4u
#define FW_LCD_LINE_CHARS 16u

/* Command flags (Pi -> STM32). */
#define FW_CMD_FLAG_RUN_ENABLE 0x01u

#define FW_MAX_STEERING_DEG 18.0f
#define FW_MAX_SPEED_FWD 2.0f
#define FW_MAX_SPEED_REV -4.0f

#define FW_WATCHDOG_TIMEOUT_MS 250u
#define FW_CONTROL_LOOP_PERIOD_MS 1u
#define FW_TELEMETRY_PERIOD_MS 20u

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
 * Baseline STM32 peripheral bindings (NUCLEO-L432KC / NUCLEO-G431KB).
 * These symbols are expected from Cube-generated HAL files.
 *
 * Net mapping from Hat schematic:
 * - TX_MCU / RX_MCU -> D1 / D0 -> USART1
 * - PROPULSION_MCU -> D9 -> TIM1_CH1
 * - SERVO_DIR_MCU -> D10 -> TIM1_CH4
 * - FOURCHE -> A0 -> TIM2_CH1
 */
#define FW_USB_UART_HANDLE huart1          /* TX_MCU / RX_MCU */
#define FW_PWM_TIMER_HANDLE htim1          /* PROPULSION_MCU / SERVO_DIR_MCU */
#define FW_PWM_PROP_CHANNEL TIM_CHANNEL_1  /* PROPULSION_MCU */
#define FW_PWM_STEERING_CHANNEL TIM_CHANNEL_4 /* SERVO_DIR_MCU */
#define FW_WHEEL_TIMER_HANDLE htim2        /* FOURCHE */
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
/*
 * Board-specific ADC instance for CAPT_IR_D (A2/PA3):
 * - NUCLEO-L432KC: hadc1
 * - NUCLEO-G431KB (PA3/ADC1_IN4 baseline): hadc1
 */
#if defined(STM32L432xx)
#define FW_REAR_OBSTACLE_ADC_HANDLE hadc1
#elif defined(STM32G431xx)
#define FW_REAR_OBSTACLE_ADC_HANDLE hadc1
#else
#error "Unsupported STM32 target: define CAPT_IR_D ADC handle for this board."
#endif
#define FW_REAR_OBSTACLE_ADC_THRESHOLD_RAW 2048u /* 12-bit ADC range: 0..4095 */
#define FW_REAR_OBSTACLE_ACTIVE_HIGH 1u

/* Optional logging (set to 1 if UART is configured and desired). */
#define FW_LOG_USE_UART 0u
#define FW_LOG_UART_HANDLE huart1

/*
 * OLED text display (mezzanine OLED1) over STM32 I2C.
 * Set FW_OLED_ENABLE to 0 if OLED is not present on your board.
 */
#define FW_OLED_ENABLE 0u
#define FW_OLED_I2C_HANDLE hi2c1
#define FW_OLED_I2C_TIMEOUT_MS 5u

#endif /* FW_CONFIG_H */
