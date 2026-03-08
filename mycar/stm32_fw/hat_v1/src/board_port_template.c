#include "board_if.h"

/*
 * This is a template only.
 * Replace every function body with STM32 HAL code from your CubeIDE project.
 */

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
    /*
     * TODO: read rear obstacle from IR sensor input (typically CAPT_IR_D on PA3),
     * with threshold/polarity validated on hardware.
     */
    return false;
}

void Board_Log(const char *message)
{
    (void)message;
    /* TODO: optional UART/SWO logging implementation. */
}
