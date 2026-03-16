#include "board_if.h"
#include "fw_app.h"
#include "fw_config.h"

/*
 * Reference loop for CubeIDE integration.
 * Merge this logic into your generated main.c after HAL/Cube init.
 */
int main(void)
{
    fw_app_t app;
    uint8_t rx_frame[FW_PROTOCOL_FRAME_SIZE] = {0u};
    uint8_t tx_frame[FW_PROTOCOL_FRAME_SIZE] = {0u};
    fw_pwm_output_t pwm = {FW_PWM_PROP_STOP, FW_PWM_STEERING_CENTER};
    fw_telemetry_t telemetry = {0.0f, false, 0u};
    bool watchdog_forced_stop = false;
    uint32_t now_ms;

    Board_Init();
    now_ms = Board_Millis();
    FwApp_Init(&app, now_ms);

    for (;;)
    {
        now_ms = Board_Millis();

        if (Board_SpiReadFrame(rx_frame))
        {
            (void)FwApp_OnSpiFrame(&app, rx_frame, now_ms);
        }

        telemetry.wheel_speed_m_s = Board_ReadWheelSpeedMps();
        telemetry.rear_obstacle = Board_ReadRearObstacle();
        telemetry.flags = 0u;

        FwApp_BuildOutputs(
            &app,
            &telemetry,
            now_ms,
            &pwm,
            tx_frame,
            &watchdog_forced_stop);

        Board_SetPropulsionDuty(pwm.propulsion_duty);
        Board_SetSteeringDuty(pwm.steering_duty);
        Board_SpiSetReplyFrame(tx_frame);

        if (watchdog_forced_stop)
        {
            Board_Log("watchdog:forced_stop");
        }

        Board_DelayMs(FW_CONTROL_LOOP_PERIOD_MS);
    }
}
