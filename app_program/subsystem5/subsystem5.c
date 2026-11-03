/*
 *	subsystem5.c
 *	Subsystem 5 -- <name>.  EMPTY TEMPLATE.
 *
 *	subsystem5_init():  create this subsystem's kernel objects
 *	                    (semaphores, mailboxes, ...).  Do not start
 *	                    tasks here.
 *	subsystem5_start(): create and start this subsystem's tasks, e.g.
 *	    ID tid = app_start_task(my_task, S5_PRI_MAIN, 0, "my_task");
 *	    return (tid < E_OK) ? tid : E_OK;
 *
 *	Drivers for this subsystem's hardware go in this folder as
 *	drv_<name>.c / drv_<name>.h, called only from the task that owns
 *	the peripheral (TEAM.md section 3).
 *
 *	Record every pin, task, object and interface in TEAM.md.
 */

#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"
#define COS_45_DEG   0.70710678f

ID flg_s5_obstacle;

LOCAL void s5_scan_task(INT stacd, void *param)
{
    ultrasonic_init();
    servo_init();

    while (1) {
        /* 1. Normal forward observation */
        servo_set_angle(SERVO_ANGLE_CENTER);
        tk_dly_tsk(150); /* Servo settle time */

        /* Read forward distance */
        float d_center = ultrasonic_read_distance_cm();
        tk_dly_tsk(60);  /* Ultrasonic acoustic decay holdoff */

        if (d_center > 0.0f && d_center < S5_WARN_DIST_CM) {
            /* Obstacle ahead: signal warning */
            tk_set_flg(flg_s5_obstacle, S5_FLAG_SLOWDOWN);

            /* 2. Coarse sweep: Measure Left */
            servo_set_angle(SERVO_ANGLE_LEFT);
            tk_dly_tsk(150);
            float d_left = ultrasonic_read_distance_cm();
            tk_dly_tsk(60); /* Transducer holdoff before next burst */

            /* 3. Coarse sweep: Measure Right */
            servo_set_angle(SERVO_ANGLE_RIGHT);
            tk_dly_tsk(200);
            float d_right = ultrasonic_read_distance_cm();
            tk_dly_tsk(60);

            /* Return to center and settle */
            servo_set_angle(SERVO_ANGLE_CENTER);
            tk_dly_tsk(150);

            /* 4. Choose direction using clearance helper */
            S5_BYPASS_DIR dir = s5_calculate_clearance(d_left, d_right);
            if (dir == S5_DIR_LEFT) {
                tk_set_flg(flg_s5_obstacle, S5_FLAG_AVOID_LEFT);
            } else {
                tk_set_flg(flg_s5_obstacle, S5_FLAG_AVOID_RIGHT);
            }

            if (d_center < S5_STOP_DIST_CM) {
                tk_set_flg(flg_s5_obstacle, S5_FLAG_EMERGENCY);
            }
        } else {
            tk_set_flg(flg_s5_obstacle, S5_FLAG_CLEAR);
        }

        tk_dly_tsk(100);
    }
}

EXPORT ER subsystem5_init(void)
{
    T_CFLG cflg = {
        .flgatr = TA_WMUL | TA_TFIFO,
        .iflgptn = S5_FLAG_CLEAR
    };
    flg_s5_obstacle = tk_cre_flg(&cflg);
    return (flg_s5_obstacle < E_OK) ? flg_s5_obstacle : E_OK;
}

EXPORT ER subsystem5_start(void)
{
    ID tid = app_start_task(s5_scan_task, S5_PRI_MAIN, 0, "s5_scan");
    return (tid < E_OK) ? tid : E_OK;
}

/* Lightweight square root using Newton-Raphson (no <math.h> / libm dependency) */
static float fast_sqrtf(float val)
{
    if (val <= 0.0f) return 0.0f;
    float x = val;
    for (int i = 0; i < 10; i++) {
        x = 0.5f * (x + (val / x));
    }
    return x;
}

/* Determine bypass direction based on side clearance */
S5_BYPASS_DIR s5_calculate_clearance(float d_left, float d_right)
{
    /* If both timed out (open air), default right */
    if (d_left < 0.0f && d_right < 0.0f) {
        return S5_DIR_RIGHT;
    }
    /* If one side has no obstacle reflection, pick the open side */
    if (d_left < 0.0f) return S5_DIR_LEFT;
    if (d_right < 0.0f) return S5_DIR_RIGHT;

    /* Pick the side with larger open clearance */
    return (d_left > d_right) ? S5_DIR_LEFT : S5_DIR_RIGHT;
}

/* Law of Cosines at 45 deg:
 * W^2 = d_c^2 + d_s^2 - 2 * d_c * d_s * cos(45 deg) */
float s5_estimate_width_cm(float d_center, float d_side, float angle_deg)
{
    (void)angle_deg; /* Standardized to 45 deg sweep */

    if (d_center <= 0.0f || d_side <= 0.0f) {
        return 0.0f;
    }

    float w_sq = (d_center * d_center) + (d_side * d_side) 
                 - (2.0f * d_center * d_side * COS_45_DEG);

    return (w_sq > 0.0f) ? fast_sqrtf(w_sq) : 0.0f;
}