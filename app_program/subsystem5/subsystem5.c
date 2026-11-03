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

ID flg_s5_obstacle;

LOCAL void s5_scan_task(INT stacd, void *param)
{
    ultrasonic_init();
    servo_init();

    while (1) {
        /* 1. Normal forward observation */
        servo_set_angle(SERVO_ANGLE_CENTER);
        tk_dly_tsk(60); /* wait for servo settling */
        float d_center = ultrasonic_read_distance_cm();

        if (d_center > 0.0f && d_center < S5_WARN_DIST_CM) {
            /* Obstacle ahead: slow down or stop */
            tk_set_flg(flg_s5_obstacle, S5_FLAG_SLOWDOWN);

            /* 2. Coarse sweep: Measure Left */
            servo_set_angle(SERVO_ANGLE_LEFT);
            tk_dly_tsk(150);
            float d_left = ultrasonic_read_distance_cm();

            /* 3. Coarse sweep: Measure Right */
            servo_set_angle(SERVO_ANGLE_RIGHT);
            tk_dly_tsk(200);
            float d_right = ultrasonic_read_distance_cm();

            /* Return to center */
            servo_set_angle(SERVO_ANGLE_CENTER);

            /* 4. Choose direction based on clearance */
            if (d_left > d_right) {
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