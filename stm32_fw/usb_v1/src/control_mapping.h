#ifndef CONTROL_MAPPING_H
#define CONTROL_MAPPING_H

#include "fw_protocol.h"

typedef struct
{
    float propulsion_duty;
    float steering_duty;
} fw_pwm_output_t;

void FwControl_CommandToPwm(
    const fw_drive_command_t *cmd,
    fw_pwm_output_t *out_pwm);

#endif /* CONTROL_MAPPING_H */
