#include "control_mapping.h"

#include "fw_config.h"

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

void FwControl_CommandToPwm(
    const fw_drive_command_t *cmd,
    fw_pwm_output_t *out_pwm)
{
    float speed;
    float steer;
    float delta;
    float steer_pwm;

    if (cmd == 0 || out_pwm == 0)
    {
        return;
    }

    speed = clampf(cmd->speed_m_s, -FW_PWM_SPEED_HARD, FW_PWM_SPEED_HARD);
    steer = clampf(cmd->steering_deg, -FW_MAX_STEERING_DEG, FW_MAX_STEERING_DEG);

    if (speed == 0.0f)
    {
        out_pwm->propulsion_duty = FW_PWM_PROP_STOP;
    }
    else if (speed > 0.0f)
    {
        delta = (speed * FW_PWM_PROP_DELTA_MAX) / FW_PWM_SPEED_HARD;
        out_pwm->propulsion_duty = FW_PWM_PROP_STOP +
                                   FW_PWM_DIRECTION_SIGN * (FW_PWM_PROP_DEADBAND + delta);
    }
    else
    {
        delta = (speed * FW_PWM_PROP_DELTA_MAX) / FW_PWM_SPEED_HARD;
        out_pwm->propulsion_duty = FW_PWM_PROP_STOP -
                                   FW_PWM_DIRECTION_SIGN * (FW_PWM_PROP_DEADBAND - delta);
    }

    steer_pwm = FW_PWM_STEERING_CENTER +
                FW_PWM_DIRECTION_SIGN *
                    ((FW_PWM_STEERING_MAX - FW_PWM_STEERING_MIN) * steer /
                     (2.0f * FW_MAX_STEERING_DEG));

    out_pwm->steering_duty = clampf(steer_pwm, FW_PWM_STEERING_MIN, FW_PWM_STEERING_MAX);
}
