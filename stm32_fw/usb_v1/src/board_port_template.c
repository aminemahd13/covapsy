#include "board_if.h"

#include <string.h>

#include "main.h"
#include "tim.h"
#include "usart.h"

#if FW_REAR_OBSTACLE_USE_ADC
#include "adc.h"
#endif

#if FW_LOG_USE_UART
#include "usart.h"
#endif

/*
 * Baseline board port for NUCLEO-G431KB using USB serial command transport.
 * Adapt handles/channels in fw_config.h if your Cube project differs.
 */

static char s_usb_rx_accum[FW_PROTOCOL_LINE_MAX] = {0};
static uint32_t s_usb_rx_len = 0u;

static volatile float s_wheel_speed_m_s = 0.0f;
static uint32_t s_wheel_last_capture = 0u;
static bool s_wheel_capture_initialized = false;

static float clampf(float value, float min_v, float max_v)
{
    if (value < min_v)
    {
        return min_v;
    }
    if (value > max_v)
    {
        return max_v;
    }
    return value;
}

static bool Board_IsWheelActiveChannel(const TIM_HandleTypeDef *htim)
{
#if FW_WHEEL_TIMER_CHANNEL == TIM_CHANNEL_1
    return htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1;
#elif FW_WHEEL_TIMER_CHANNEL == TIM_CHANNEL_2
    return htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2;
#elif FW_WHEEL_TIMER_CHANNEL == TIM_CHANNEL_3
    return htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3;
#elif FW_WHEEL_TIMER_CHANNEL == TIM_CHANNEL_4
    return htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4;
#else
    return false;
#endif
}

static uint32_t Board_DutyPercentToCompare(float duty_percent)
{
    float clipped = clampf(duty_percent, 0.0f, 100.0f);
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&FW_PWM_TIMER_HANDLE);
    float compare = (clipped * (float)(period + 1u)) / 100.0f;
    if (compare > (float)period)
    {
        compare = (float)period;
    }
    return (uint32_t)(compare + 0.5f);
}

static bool Board_ApplyRearObstaclePolarity(bool raw_detected)
{
#if FW_REAR_OBSTACLE_ACTIVE_HIGH
    return raw_detected;
#else
    return !raw_detected;
#endif
}

void Board_Init(void)
{
    (void)HAL_TIM_PWM_Start(&FW_PWM_TIMER_HANDLE, FW_PWM_PROP_CHANNEL);
    (void)HAL_TIM_PWM_Start(&FW_PWM_TIMER_HANDLE, FW_PWM_STEERING_CHANNEL);

    Board_SetPropulsionDuty(FW_PWM_PROP_STOP);
    Board_SetSteeringDuty(FW_PWM_STEERING_CENTER);

    (void)HAL_TIM_Base_Start_IT(&FW_WHEEL_TIMER_HANDLE);
    (void)HAL_TIM_IC_Start_IT(&FW_WHEEL_TIMER_HANDLE, FW_WHEEL_TIMER_CHANNEL);
}

uint32_t Board_Millis(void)
{
    return HAL_GetTick();
}

void Board_DelayMs(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

bool Board_UsbReadLine(char *out_line, uint32_t out_line_size)
{
    uint8_t rx_byte;

    if (out_line == 0 || out_line_size == 0u)
    {
        return false;
    }

    while (HAL_UART_Receive(&FW_USB_UART_HANDLE, &rx_byte, 1u, 0u) == HAL_OK)
    {
        if (rx_byte == '\r')
        {
            continue;
        }

        if (rx_byte == '\n')
        {
            uint32_t copy_len;
            if (s_usb_rx_len == 0u)
            {
                continue;
            }

            copy_len = s_usb_rx_len;
            if (copy_len >= out_line_size)
            {
                copy_len = out_line_size - 1u;
            }

            (void)memcpy(out_line, s_usb_rx_accum, copy_len);
            out_line[copy_len] = '\0';
            s_usb_rx_len = 0u;
            return true;
        }

        if (s_usb_rx_len < (FW_PROTOCOL_LINE_MAX - 1u))
        {
            s_usb_rx_accum[s_usb_rx_len++] = (char)rx_byte;
        }
        else
        {
            /* Overflow: drop partial line and wait for next newline. */
            s_usb_rx_len = 0u;
        }
    }

    return false;
}

void Board_UsbWriteLine(const char *line)
{
    size_t len;

    if (line == 0)
    {
        return;
    }

    len = strlen(line);
    if (len == 0u)
    {
        return;
    }

    (void)HAL_UART_Transmit(
        &FW_USB_UART_HANDLE,
        (uint8_t *)line,
        (uint16_t)len,
        2u);
}

void Board_SetPropulsionDuty(float duty_percent)
{
    __HAL_TIM_SET_COMPARE(
        &FW_PWM_TIMER_HANDLE,
        FW_PWM_PROP_CHANNEL,
        Board_DutyPercentToCompare(duty_percent));
}

void Board_SetSteeringDuty(float duty_percent)
{
    __HAL_TIM_SET_COMPARE(
        &FW_PWM_TIMER_HANDLE,
        FW_PWM_STEERING_CHANNEL,
        Board_DutyPercentToCompare(duty_percent));
}

float Board_ReadWheelSpeedMps(void)
{
    return clampf(
        s_wheel_speed_m_s,
        -FW_WHEEL_SPEED_MAX_ABS_MPS,
        FW_WHEEL_SPEED_MAX_ABS_MPS);
}

bool Board_ReadRearObstacle(void)
{
#if FW_REAR_OBSTACLE_USE_ADC
    uint32_t adc_raw = 0u;

    if (HAL_ADC_Start(&FW_REAR_OBSTACLE_ADC_HANDLE) != HAL_OK)
    {
        return false;
    }

    if (HAL_ADC_PollForConversion(&FW_REAR_OBSTACLE_ADC_HANDLE, 1u) == HAL_OK)
    {
        adc_raw = HAL_ADC_GetValue(&FW_REAR_OBSTACLE_ADC_HANDLE);
    }

    (void)HAL_ADC_Stop(&FW_REAR_OBSTACLE_ADC_HANDLE);

    return Board_ApplyRearObstaclePolarity(
        adc_raw >= FW_REAR_OBSTACLE_ADC_THRESHOLD_RAW);
#elif defined(CAPT_IR_D_GPIO_Port) && defined(CAPT_IR_D_Pin)
    return Board_ApplyRearObstaclePolarity(
        HAL_GPIO_ReadPin(CAPT_IR_D_GPIO_Port, CAPT_IR_D_Pin) == GPIO_PIN_SET);
#else
    return false;
#endif
}

void Board_Log(const char *message)
{
#if FW_LOG_USE_UART
    static const uint8_t newline[2] = {'\r', '\n'};
    size_t len;

    if (message == 0)
    {
        return;
    }

    len = strlen(message);
    if (len > 0u)
    {
        (void)HAL_UART_Transmit(
            &FW_LOG_UART_HANDLE,
            (uint8_t *)message,
            (uint16_t)len,
            5u);
    }
    (void)HAL_UART_Transmit(
        &FW_LOG_UART_HANDLE,
        (uint8_t *)newline,
        (uint16_t)sizeof(newline),
        5u);
#else
    (void)message;
#endif
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint32_t capture_now;
    uint32_t delta_ticks;
    float distance_per_pulse_m;
    float speed_m_s;

    if (htim != &FW_WHEEL_TIMER_HANDLE)
    {
        return;
    }
    if (!Board_IsWheelActiveChannel(htim))
    {
        return;
    }

    capture_now = HAL_TIM_ReadCapturedValue(&FW_WHEEL_TIMER_HANDLE, FW_WHEEL_TIMER_CHANNEL);
    if (!s_wheel_capture_initialized)
    {
        s_wheel_last_capture = capture_now;
        s_wheel_capture_initialized = true;
        return;
    }

    delta_ticks = capture_now - s_wheel_last_capture;
    s_wheel_last_capture = capture_now;
    if (delta_ticks == 0u)
    {
        return;
    }

    distance_per_pulse_m =
        (FW_WHEEL_DISTANCE_PER_TURN_MM / FW_WHEEL_PULSES_PER_TURN) / 1000.0f;
    speed_m_s = (distance_per_pulse_m * FW_WHEEL_TIMER_TICK_HZ) / (float)delta_ticks;
    s_wheel_speed_m_s = clampf(
        speed_m_s,
        -FW_WHEEL_SPEED_MAX_ABS_MPS,
        FW_WHEEL_SPEED_MAX_ABS_MPS);
}
