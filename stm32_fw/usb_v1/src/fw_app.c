#include "fw_app.h"

#include <string.h>

#include "fw_config.h"

static fw_drive_command_t zero_command(void)
{
    fw_drive_command_t cmd;
    cmd.steering_deg = 0.0f;
    cmd.speed_m_s = 0.0f;
    cmd.flags = 0u;
    cmd.emergency_brake = false;
    cmd.seq = 0u;
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
    app->last_rx_seq = 0u;
    app->command_received = false;
}

bool FwApp_OnCommandLine(
    fw_app_t *app,
    const char *rx_line,
    uint32_t now_ms)
{
    fw_drive_command_t decoded;
    if (app == 0)
    {
        return false;
    }

    if (!FwProtocol_ParseCommandLine(rx_line, &decoded))
    {
        return false;
    }

    app->last_command = decoded;
    app->last_rx_seq = decoded.seq;
    app->last_command_ms = now_ms;
    app->command_received = true;
    return true;
}

void FwApp_BuildOutputs(
    fw_app_t *app,
    const fw_telemetry_t *telemetry,
    uint32_t now_ms,
    fw_pwm_output_t *out_pwm,
    char *out_tx_line,
    uint32_t out_tx_line_size,
    bool *watchdog_forced_stop)
{
    fw_drive_command_t effective_cmd;
    uint32_t elapsed_ms;
    fw_telemetry_t local_telemetry;
    int32_t status_code;

    if (app == 0 || out_pwm == 0 || out_tx_line == 0 || out_tx_line_size == 0u)
    {
        return;
    }
    out_tx_line[0] = '\0';

    effective_cmd = app->last_command;
    elapsed_ms = now_ms - app->last_command_ms;
    status_code = 0;

    if (watchdog_forced_stop != 0)
    {
        *watchdog_forced_stop = false;
    }

    if (!app->command_received || elapsed_ms > FW_WATCHDOG_TIMEOUT_MS)
    {
        effective_cmd = zero_command();
        status_code = 1;
        if (watchdog_forced_stop != 0)
        {
            *watchdog_forced_stop = true;
        }
    }

    /* Competition guard: movement is enabled only when RUN flag is asserted. */
    if ((effective_cmd.flags & FW_CMD_FLAG_RUN_ENABLE) == 0u)
    {
        effective_cmd = zero_command();
        if (status_code == 0)
        {
            status_code = 2;
        }
    }
    if (app->last_command.emergency_brake)
    {
        effective_cmd = zero_command();
        status_code = 3;
    }

    FwControl_CommandToPwm(&effective_cmd, out_pwm);

    if (telemetry == 0)
    {
        local_telemetry.wheel_speed_m_s = 0.0f;
        local_telemetry.rear_obstacle = false;
        local_telemetry.flags = 0u;
        if (!FwProtocol_FormatTelemetryLine(
            &local_telemetry,
            app->last_rx_seq,
            status_code,
            out_tx_line,
            out_tx_line_size))
        {
            (void)strncpy(out_tx_line, "TEL,0,0.000,0,99\n", out_tx_line_size - 1u);
            out_tx_line[out_tx_line_size - 1u] = '\0';
        }
        return;
    }

    if (!FwProtocol_FormatTelemetryLine(
        telemetry,
        app->last_rx_seq,
        status_code,
        out_tx_line,
        out_tx_line_size))
    {
        (void)strncpy(out_tx_line, "TEL,0,0.000,0,99\n", out_tx_line_size - 1u);
        out_tx_line[out_tx_line_size - 1u] = '\0';
    }
}
