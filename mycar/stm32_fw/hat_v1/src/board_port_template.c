#include "board_if.h"
#include "main.h"

#if FW_REAR_OBSTACLE_USE_ADC
#include "adc.h"
#endif

/*
 * This is a template only.
 * Replace every function body with STM32 HAL code from your CubeIDE project.
 */

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
    /* TODO: call MX_* init functions and set neutral PWM outputs. */
}

uint32_t Board_Millis(void)
{
    /* TODO: return HAL_GetTick() or equivalent monotonic millisecond counter. */
    return 0u;
}

void Board_DelayMs(uint32_t delay_ms)
{
    (void)delay_ms;
    /* TODO: call HAL_Delay(delay_ms) or scheduler-safe equivalent. */
}

bool Board_SpiReadFrame(uint8_t out_frame[FW_PROTOCOL_FRAME_SIZE])
{
    (void)out_frame;
    /*
     * TODO:
     *  - collect 6-byte transfer from SPI3 slave interface (PB3/PB4/PB5)
     *  - configure SPI with software NSS (HAT baseline has no wired CS to STM32)
     *  - return true exactly once per complete received frame
     */
    return false;
}

void Board_SpiSetReplyFrame(const uint8_t frame[FW_PROTOCOL_FRAME_SIZE])
{
    (void)frame;
    /* TODO: queue reply bytes for next SPI transfer. */
}

void Board_SetPropulsionDuty(float duty_percent)
{
    (void)duty_percent;
    /* TODO: map duty percent to timer compare register for propulsion channel. */
}

void Board_SetSteeringDuty(float duty_percent)
{
    (void)duty_percent;
    /* TODO: map duty percent to timer compare register for steering channel. */
}

float Board_ReadWheelSpeedMps(void)
{
    /* TODO: compute wheel speed from FOURCHE input (PA0/TIM2_CH1) and return m/s. */
    return 0.0f;
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
    /*
     * No rear obstacle source configured in this project.
     * Define CAPT_IR_D pin labels in CubeMX or enable ADC mode above.
     */
    return false;
#endif
}

void Board_Log(const char *message)
{
    (void)message;
    /* TODO: optional UART/SWO logging implementation. */
}
