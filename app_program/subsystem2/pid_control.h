#ifndef SPEED_PID_H
#define SPEED_PID_H

float speed_p_update(float target_speed, float measured_speed,
                     float base_pwm, float kp);

#endif