#ifndef PID_CONTROL_H
#define PID_CONTROL_H

/* P Controller */

float speed_p_update(float target_speed, float measured_speed,
                     float base_pwm, float kp);

/* PI Controller */

typedef struct {
    float kp;
    float ki;
    float integral;
    float base_pwm;
} SpeedPI;

void speed_pi_init(SpeedPI *controller, float kp, float ki,
                   float base_pwm);

float speed_pi_update(SpeedPI *controller, float target_speed,
                      float measured_speed, float dt_seconds);

/* Full PID Controller */
typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float previous_error;
    float base_pwm;

    int first_update;
} SpeedPID;

void speed_pid_init(SpeedPID *controller, float kp, float ki,
                    float kd, float base_pwm);

float speed_pid_update(SpeedPID *controller, float target_speed,
                       float measured_speed, float dt_seconds);

#endif /* PID_CONTROL_H */