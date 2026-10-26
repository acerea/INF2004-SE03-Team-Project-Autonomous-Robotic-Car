#include "pid_control.h"

float speed_p_update(float target_speed, float measured_speed,
                     float base_pwm, float kp)
{
    float error = target_speed - measured_speed;
    float pwm = base_pwm + kp * error;

    if (pwm > 100.0f) {
        pwm = 100.0f;
    }

    if (pwm < 0.0f) {
        pwm = 0.0f;
    }

    return pwm;
}