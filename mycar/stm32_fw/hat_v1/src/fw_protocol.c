#include "fw_protocol.h"

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

uint8_t FwProtocol_EncodeSignedU8(float value, float min_v, float max_v)
{
    float clipped;
    float normalized;
    float scaled;
    int rounded;

    if (max_v <= min_v)
    {
        return 127u;
    }

    clipped = clampf(value, min_v, max_v);
    normalized = (clipped - min_v) / (max_v - min_v);
    scaled = normalized * 255.0f;
    rounded = (int)(scaled + 0.5f);

    if (rounded < 0)
    {
        rounded = 0;
    }
    if (rounded > 255)
    {
        rounded = 255;
    }

    return (uint8_t)rounded;
}

float FwProtocol_DecodeSignedU8(uint8_t byte_val, float min_v, float max_v)
{
    float normalized;
    if (max_v <= min_v)
    {
        return 0.0f;
    }
    normalized = ((float)byte_val) / 255.0f;
    return min_v + normalized * (max_v - min_v);
}

uint8_t FwProtocol_Checksum(const uint8_t frame[FW_PROTOCOL_FRAME_SIZE])
{
    uint32_t sum = 0u;
    uint8_t i;
    for (i = 0u; i < FW_PROTOCOL_CHECKSUM_INDEX; i++)
    {
        sum += frame[i];
    }
    return (uint8_t)(sum & 0xFFu);
}

bool FwProtocol_IsValidFrame(const uint8_t frame[FW_PROTOCOL_FRAME_SIZE])
{
    if (frame[0] != FW_PROTOCOL_HEADER0 || frame[1] != FW_PROTOCOL_HEADER1)
    {
        return false;
    }
    return frame[FW_PROTOCOL_CHECKSUM_INDEX] == FwProtocol_Checksum(frame);
}

bool FwProtocol_DecodeDriveFrame(
    const uint8_t frame[FW_PROTOCOL_FRAME_SIZE],
    fw_drive_command_t *out_cmd)
{
    if (out_cmd == 0)
    {
        return false;
    }

    if (!FwProtocol_IsValidFrame(frame))
    {
        return false;
    }

    out_cmd->steering_deg = FwProtocol_DecodeSignedU8(
        frame[FW_PROTOCOL_STEERING_INDEX],
        -FW_MAX_STEERING_DEG,
        FW_MAX_STEERING_DEG);

    out_cmd->speed_m_s = FwProtocol_DecodeSignedU8(
        frame[FW_PROTOCOL_SPEED_INDEX],
        FW_MAX_SPEED_REV,
        FW_MAX_SPEED_FWD);

    out_cmd->flags = frame[FW_PROTOCOL_FLAGS_INDEX];
    return true;
}

void FwProtocol_EncodeTelemetryFrame(
    const fw_telemetry_t *telemetry,
    uint8_t out_frame[FW_PROTOCOL_FRAME_SIZE])
{
    if (telemetry == 0 || out_frame == 0)
    {
        return;
    }

    out_frame[0] = FW_PROTOCOL_HEADER0;
    out_frame[1] = FW_PROTOCOL_HEADER1;
    out_frame[FW_PROTOCOL_STEERING_INDEX] = 127u;
    out_frame[FW_PROTOCOL_SPEED_INDEX] = FwProtocol_EncodeSignedU8(
        telemetry->wheel_speed_m_s,
        FW_MAX_SPEED_REV,
        FW_MAX_SPEED_FWD);

    out_frame[FW_PROTOCOL_FLAGS_INDEX] = telemetry->flags;
    if (telemetry->rear_obstacle)
    {
        out_frame[FW_PROTOCOL_FLAGS_INDEX] |= 0x01u;
    }

    out_frame[FW_PROTOCOL_CHECKSUM_INDEX] = FwProtocol_Checksum(out_frame);
}
