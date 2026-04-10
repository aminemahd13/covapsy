#ifndef FW_PROTOCOL_H
#define FW_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#include "fw_config.h"

typedef struct
{
    float steering_deg;
    float speed_m_s;
    uint8_t flags;
    bool emergency_brake;
    uint32_t seq;
} fw_drive_command_t;

typedef struct
{
    float wheel_speed_m_s;
    bool rear_obstacle;
    uint8_t flags;
} fw_telemetry_t;

bool FwProtocol_ParseCommandLine(
    const char *line,
    fw_drive_command_t *out_cmd);

bool FwProtocol_FormatTelemetryLine(
    const fw_telemetry_t *telemetry,
    uint32_t seq,
    int32_t status_code,
    char *out_line,
    uint32_t out_line_size);

#endif /* FW_PROTOCOL_H */
