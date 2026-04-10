#ifndef FW_APP_H
#define FW_APP_H

#include <stdbool.h>
#include <stdint.h>

#include "control_mapping.h"
#include "fw_protocol.h"

typedef struct
{
    fw_drive_command_t last_command;
    uint32_t last_command_ms;
    uint32_t last_rx_seq;
    bool command_received;
} fw_app_t;

void FwApp_Init(fw_app_t *app, uint32_t now_ms);

bool FwApp_OnCommandLine(
    fw_app_t *app,
    const char *rx_line,
    uint32_t now_ms);

void FwApp_BuildOutputs(
    fw_app_t *app,
    const fw_telemetry_t *telemetry,
    uint32_t now_ms,
    fw_pwm_output_t *out_pwm,
    char *out_tx_line,
    uint32_t out_tx_line_size,
    bool *watchdog_forced_stop);

#endif /* FW_APP_H */
