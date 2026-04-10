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
    char rx_line[FW_PROTOCOL_LINE_MAX] = {0};
    char tx_line[FW_PROTOCOL_LINE_MAX] = {0};
    fw_pwm_output_t pwm = {FW_PWM_PROP_STOP, FW_PWM_STEERING_CENTER};
    fw_telemetry_t telemetry = {0.0f, false, 0u};
    bool watchdog_forced_stop = false;
    uint32_t now_ms;
    uint32_t last_tx_ms;

    Board_Init();
    now_ms = Board_Millis();
    last_tx_ms = now_ms;
    FwApp_Init(&app, now_ms);

    for (;;)
    {
        now_ms = Board_Millis();

        if (Board_UsbReadLine(rx_line, sizeof(rx_line)))
        {
            (void)FwApp_OnCommandLine(&app, rx_line, now_ms);
        }

        telemetry.wheel_speed_m_s = Board_ReadWheelSpeedMps();
        telemetry.rear_obstacle = Board_ReadRearObstacle();
        telemetry.flags = 0u;

        FwApp_BuildOutputs(
            &app,
            &telemetry,
            now_ms,
            &pwm,
            tx_line,
            sizeof(tx_line),
            &watchdog_forced_stop);

        Board_SetPropulsionDuty(pwm.propulsion_duty);
        Board_SetSteeringDuty(pwm.steering_duty);
        if ((now_ms - last_tx_ms) >= FW_TELEMETRY_PERIOD_MS)
        {
            Board_UsbWriteLine(tx_line);
            last_tx_ms = now_ms;
        }

        if (watchdog_forced_stop)
        {
            Board_Log("watchdog:forced_stop");
        }

        Board_DelayMs(FW_CONTROL_LOOP_PERIOD_MS);
    }
}
