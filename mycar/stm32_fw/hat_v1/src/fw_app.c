#include "fw_app.h"

#include "fw_config.h"

static fw_drive_command_t zero_command(void)
{
    fw_drive_command_t cmd;
    cmd.steering_deg = 0.0f;
    cmd.speed_m_s = 0.0f;
    cmd.flags = 0u;
    return cmd;
}

void FwApp_Init(fw_app_t *app, uint32_t now_ms)
{
    if (app == 0)
    {
        return;
    }
    app->last_command = zero_command();
    app->last_command_ms = now_ms;
    app->command_received = false;
}

bool FwApp_OnSpiFrame(
    fw_app_t *app,
    const uint8_t rx_frame[FW_PROTOCOL_FRAME_SIZE],
    uint32_t now_ms)
{
    fw_drive_command_t decoded;
    if (app == 0)
    {
        return false;
    }

    if (!FwProtocol_DecodeDriveFrame(rx_frame, &decoded))
    {
        return false;
    }

    app->last_command = decoded;
    app->last_command_ms = now_ms;
    app->command_received = true;
    return true;
}

void FwApp_BuildOutputs(
    fw_app_t *app,
    const fw_telemetry_t *telemetry,
    uint32_t now_ms,
    fw_pwm_output_t *out_pwm,
    uint8_t out_tx_frame[FW_PROTOCOL_FRAME_SIZE],
    bool *watchdog_forced_stop)
{
    fw_drive_command_t effective_cmd;
    uint32_t elapsed_ms;
    fw_telemetry_t local_telemetry;

    if (app == 0 || out_pwm == 0 || out_tx_frame == 0)
    {
        return;
    }

    effective_cmd = app->last_command;
    elapsed_ms = now_ms - app->last_command_ms;

    if (watchdog_forced_stop != 0)
    {
        *watchdog_forced_stop = false;
    }

    if (!app->command_received || elapsed_ms > FW_WATCHDOG_TIMEOUT_MS)
    {
        effective_cmd = zero_command();
        if (watchdog_forced_stop != 0)
        {
            *watchdog_forced_stop = true;
        }
    }

    FwControl_CommandToPwm(&effective_cmd, out_pwm);

    if (telemetry == 0)
    {
        local_telemetry.wheel_speed_m_s = 0.0f;
        local_telemetry.rear_obstacle = false;
        local_telemetry.flags = 0u;
        FwProtocol_EncodeTelemetryFrame(&local_telemetry, out_tx_frame);
        return;
    }

    FwProtocol_EncodeTelemetryFrame(telemetry, out_tx_frame);
}
