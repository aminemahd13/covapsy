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
} fw_drive_command_t;

typedef struct
{
    float wheel_speed_m_s;
    bool rear_obstacle;
    uint8_t flags;
} fw_telemetry_t;

uint8_t FwProtocol_EncodeSignedU8(float value, float min_v, float max_v);
float FwProtocol_DecodeSignedU8(uint8_t byte_val, float min_v, float max_v);

uint8_t FwProtocol_Checksum(const uint8_t frame[FW_PROTOCOL_FRAME_SIZE]);
bool FwProtocol_IsValidFrame(const uint8_t frame[FW_PROTOCOL_FRAME_SIZE]);

bool FwProtocol_DecodeDriveFrame(
    const uint8_t frame[FW_PROTOCOL_FRAME_SIZE],
    fw_drive_command_t *out_cmd);

void FwProtocol_EncodeTelemetryFrame(
    const fw_telemetry_t *telemetry,
    uint8_t out_frame[FW_PROTOCOL_FRAME_SIZE]);

#endif /* FW_PROTOCOL_H */
