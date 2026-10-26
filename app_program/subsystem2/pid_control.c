#include "pid_control.h"

/* P Controller */
float speed_p_update(float target_speed, float measured_speed,
                     float base_pwm, float kp)
{
    float error = target_speed - measured_speed;
    float pwm = base_pwm + kp * error;

    if (pwm > 100.0f) pwm = 100.0f;
    if (pwm < 0.0f) pwm = 0.0f;

    return pwm;
}

/* PI Controller */
void speed_pi_init(SpeedPI *controller, float kp, float ki,
                   float base_pwm)
{
    controller->kp = kp;
    controller->ki = ki;
    controller->integral = 0.0f;
    controller->base_pwm = base_pwm;
}

float speed_pi_update(SpeedPI *controller, float target_speed,
                      float measured_speed, float dt_seconds)
{
    float error, proposed_integral, pwm;

    if (dt_seconds <= 0.0f) {
        return controller->base_pwm;
    }

    error = target_speed - measured_speed;

    proposed_integral = controller->integral
                      + error * dt_seconds;

    pwm = controller->base_pwm
        + controller->kp * error
        + controller->ki * proposed_integral;

    /* PWM clamping with basic anti-windup */
    if (pwm > 100.0f) {
        pwm = 100.0f;
        if (error < 0.0f)
            controller->integral = proposed_integral;
    }
    else if (pwm < 0.0f) {
        pwm = 0.0f;
        if (error > 0.0f)
            controller->integral = proposed_integral;
    }
    else {
        controller->integral = proposed_integral;
    }

    return pwm;
}

/* Full PID Controller */
void speed_pid_init(SpeedPID *controller, float kp, float ki,
                    float kd, float base_pwm)
{
    controller->kp = kp;
    controller->ki = ki;
    controller->kd = kd;

    controller->integral = 0.0f;
    controller->previous_error = 0.0f;
    controller->base_pwm = base_pwm;
    controller->first_update = 1;
}

float speed_pid_update(SpeedPID *controller, float target_speed,
                       float measured_speed, float dt_seconds)
{
    float error, derivative, proposed_integral, pwm;

    if (dt_seconds <= 0.0f) {
        return controller->base_pwm;
    }

    error = target_speed - measured_speed;

    /* Derivative term */
    if (controller->first_update) {
        derivative = 0.0f;
        controller->first_update = 0;
    }
    else {
        derivative = (error - controller->previous_error)
                   / dt_seconds;
    }

    /* Integral term */
    proposed_integral = controller->integral
                      + error * dt_seconds;

    /* PID calculation */
    pwm = controller->base_pwm
        + controller->kp * error
        + controller->ki * proposed_integral
        + controller->kd * derivative;

    /* PWM clamping with basic anti-windup */
    if (pwm > 100.0f) {
        pwm = 100.0f;
        if (error < 0.0f)
            controller->integral = proposed_integral;
    }
    else if (pwm < 0.0f) {
        pwm = 0.0f;
        if (error > 0.0f)
            controller->integral = proposed_integral;
    }
    else {
        controller->integral = proposed_integral;
    }

    controller->previous_error = error;

    return pwm;
}