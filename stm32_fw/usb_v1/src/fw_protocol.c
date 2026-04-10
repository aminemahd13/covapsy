#include "fw_protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

static bool parse_bool_token(const char *token, bool *out_value)
{
    if (token == 0 || out_value == 0)
    {
        return false;
    }

    if ((strcmp(token, "1") == 0) || (strcmp(token, "true") == 0) || (strcmp(token, "TRUE") == 0))
    {
        *out_value = true;
        return true;
    }
    if ((strcmp(token, "0") == 0) || (strcmp(token, "false") == 0) || (strcmp(token, "FALSE") == 0))
    {
        *out_value = false;
        return true;
    }
    return false;
}

static bool parse_u32_token(const char *token, uint32_t *out_value)
{
    unsigned long value;
    char *end_ptr = 0;
    if (token == 0 || out_value == 0)
    {
        return false;
    }

    value = strtoul(token, &end_ptr, 10);
    if (end_ptr == token || *end_ptr != '\0')
    {
        return false;
    }
    *out_value = (uint32_t)value;
    return true;
}

static bool parse_float_token(const char *token, float *out_value)
{
    float value;
    char *end_ptr = 0;
    if (token == 0 || out_value == 0)
    {
        return false;
    }
    value = strtof(token, &end_ptr);
    if (end_ptr == token || *end_ptr != '\0')
    {
        return false;
    }
    *out_value = value;
    return true;
}

bool FwProtocol_ParseCommandLine(
    const char *line,
    fw_drive_command_t *out_cmd)
{
    char local[FW_PROTOCOL_LINE_MAX];
    char *token = 0;
    char *tokens[6] = {0};
    uint8_t token_count = 0u;
    bool run_enable = false;
    bool emergency_brake = false;
    float steering_deg;
    float speed_m_s;
    uint32_t seq;

    if (line == 0 || out_cmd == 0)
    {
        return false;
    }

    strncpy(local, line, sizeof(local) - 1u);
    local[sizeof(local) - 1u] = '\0';

    token = strtok(local, ",");
    while (token != 0 && token_count < 6u)
    {
        tokens[token_count++] = token;
        token = strtok(0, ",");
    }

    if (token_count != 6u)
    {
        return false;
    }
    if (strcmp(tokens[0], "CMD") != 0)
    {
        return false;
    }

    if (!parse_u32_token(tokens[1], &seq))
    {
        return false;
    }
    if (!parse_float_token(tokens[2], &steering_deg))
    {
        return false;
    }
    if (!parse_float_token(tokens[3], &speed_m_s))
    {
        return false;
    }
    if (!parse_bool_token(tokens[4], &run_enable))
    {
        return false;
    }
    if (!parse_bool_token(tokens[5], &emergency_brake))
    {
        return false;
    }

    out_cmd->seq = seq;
    out_cmd->steering_deg = clampf(steering_deg, -FW_MAX_STEERING_DEG, FW_MAX_STEERING_DEG);
    out_cmd->speed_m_s = clampf(speed_m_s, FW_MAX_SPEED_REV, FW_MAX_SPEED_FWD);
    out_cmd->flags = run_enable ? FW_CMD_FLAG_RUN_ENABLE : 0u;
    out_cmd->emergency_brake = emergency_brake;

    if (out_cmd->emergency_brake)
    {
        out_cmd->speed_m_s = 0.0f;
        out_cmd->flags = 0u;
    }

    return true;
}

bool FwProtocol_FormatTelemetryLine(
    const fw_telemetry_t *telemetry,
    uint32_t seq,
    int32_t status_code,
    char *out_line,
    uint32_t out_line_size)
{
    int written;

    if (telemetry == 0 || out_line == 0 || out_line_size == 0u)
    {
        return false;
    }

    written = snprintf(
        out_line,
        (size_t)out_line_size,
        "TEL,%lu,%.3f,%u,%ld\n",
        (unsigned long)seq,
        (double)telemetry->wheel_speed_m_s,
        telemetry->rear_obstacle ? 1u : 0u,
        (long)status_code);

    if (written <= 0 || (uint32_t)written >= out_line_size)
    {
        return false;
    }

    return true;
}
