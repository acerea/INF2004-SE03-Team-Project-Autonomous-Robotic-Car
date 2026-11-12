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

#include <stdint.h>
#include <stdbool.h>
#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"
#include "../subsystem1/subsystem1.h"

#define COS_45_DEG   0.70710678f
#define S5_STATE_MASK (S5_FLAG_CLEAR | S5_FLAG_SLOWDOWN | S5_FLAG_EMERGENCY)

EXPORT ID flg_s5_obstacle;

ID flg_s5_obstacle;

/* Forward prototypes */
static float fast_sqrtf(float val);

/* Helper to clear old state flags and set the new state atomically */
static void s5_set_state(UINT new_flag)
{
    tk_clr_flg(flg_s5_obstacle, ~S5_STATE_MASK);
    tk_set_flg(flg_s5_obstacle, new_flag);
}
/* ====================================================================
 * Telemetry Publishing Helpers (Buddy 5 -> Subsystem 1 MQTT)
 * ==================================================================== */

static UW
s5_cm_field (float cm, UW max_cm)
{
    UW value = 0u;

    if (cm >= (float) max_cm)
    {
        value = max_cm;
    }
    else if (cm > 0.0f)
    {
        value = (UW) (cm + 0.5f);
    }

    return (value);
}

static void
s5_publish_obstacle (UW state, float d_center)
{
    UB           frame[PROTO_MAX_FRAME_BYTES];
    proto_bits_t bits;

    proto_init(&bits, frame, sizeof(frame));
    proto_begin(&bits, PROTO_TYPE_OBSTACLE);
    proto_put(&bits, state, (UB)PROTO_OBS_STATE_BITS);
    proto_put(&bits, s5_cm_field(d_center, PROTO_OBS_CM_MAX), (UB)PROTO_OBS_CM_BITS);
    (void) s1_publish(frame, proto_end(&bits), FALSE);
}

static void
s5_publish_profile (S5_BYPASS_DIR dir, float width_cm, float d_left,
                    float d_right)
{
    UB           frame[PROTO_MAX_FRAME_BYTES];
    proto_bits_t bits;

    UW side_val = 0u;
    if (dir == S5_DIR_LEFT) {
        side_val = 1u;
    } else if (dir == S5_DIR_RIGHT) {
        side_val = 2u;
    }

    proto_init(&bits, frame, sizeof(frame));
    proto_begin(&bits, PROTO_TYPE_OBST_PROFILE);
    proto_put(&bits, side_val, (UB)PROTO_PRO_SIDE_BITS);
    proto_put(&bits, s5_cm_field(width_cm, PROTO_PRO_WIDTH_MAX), (UB)PROTO_PRO_WIDTH_BITS);
    proto_put(&bits, s5_cm_field(d_left, PROTO_OBS_CM_MAX), (UB)PROTO_PRO_LEFT_BITS);
    proto_put(&bits, s5_cm_field(d_right, PROTO_OBS_CM_MAX), (UB)PROTO_PRO_RIGHT_BITS);
    (void) s1_publish(frame, proto_end(&bits), FALSE);
}

#define S5_STATE_FLAGS  (S5_FLAG_CLEAR | S5_FLAG_SLOWDOWN | S5_FLAG_EMERGENCY)
#define S5_SIDE_FLAGS   (S5_FLAG_AVOID_LEFT | S5_FLAG_AVOID_RIGHT)

/* Telemetry state field: 0 = clear, 1 = slow down, 2 = emergency. */
#define S5_TLM_CLEAR        0u
#define S5_TLM_SLOWDOWN     1u
#define S5_TLM_EMERGENCY    2u

/* Replace the old state bit with a new one (only one state bit is set). */
static void s5_set_state_flag(UINT flag)
{
    tk_clr_flg(flg_s5_obstacle, ~(UINT)S5_STATE_FLAGS);
    tk_set_flg(flg_s5_obstacle, flag);
}

LOCAL void s5_scan_task(INT stacd, void *param)
{
    (void)stacd;
    (void)param;

    ultrasonic_init();
    servo_init();

    while (1) {
        /* 1. Normal forward observation */
        servo_set_angle(SERVO_ANGLE_CENTER);
        tk_dly_tsk(150); /* Servo settle time */

        /* Read forward distance */
        float d_center = ultrasonic_read_distance_cm();
        tk_dly_tsk(60);  /* Ultrasonic acoustic decay holdoff */

        /* 2. Decide the state first, so EMERGENCY is not delayed by the
              sweep below, and publish it every loop (also on no echo). */
        UW state = S5_TLM_CLEAR;
        if (d_center > 0.0f && d_center < S5_STOP_DIST_CM) {
            state = S5_TLM_EMERGENCY;
            s5_set_state_flag(S5_FLAG_EMERGENCY);
        } else if (d_center > 0.0f && d_center < S5_WARN_DIST_CM) {
            state = S5_TLM_SLOWDOWN;
            s5_set_state_flag(S5_FLAG_SLOWDOWN);
        } else {
            s5_set_state_flag(S5_FLAG_CLEAR);
            tk_clr_flg(flg_s5_obstacle, ~(UINT)S5_SIDE_FLAGS); /* No side */
        }
        s5_publish_obstacle(state, d_center);

        if (state != S5_TLM_CLEAR) {
            /* 3. Coarse sweep: Measure Left */
            servo_set_angle(SERVO_ANGLE_LEFT);
            tk_dly_tsk(150);
            float d_left = ultrasonic_read_distance_cm();
            tk_dly_tsk(60); /* Transducer holdoff before next burst */

            /* 4. Coarse sweep: Measure Right */
            servo_set_angle(SERVO_ANGLE_RIGHT);
            tk_dly_tsk(200);
            float d_right = ultrasonic_read_distance_cm();
            tk_dly_tsk(60);

            /* Return to center and settle */
            servo_set_angle(SERVO_ANGLE_CENTER);
            tk_dly_tsk(150);

            /* 5. Choose direction, replace the old side bit, publish */
            S5_BYPASS_DIR dir = s5_calculate_clearance(d_left, d_right);
            tk_clr_flg(flg_s5_obstacle, ~(UINT)S5_SIDE_FLAGS);
            tk_set_flg(flg_s5_obstacle, (dir == S5_DIR_LEFT)
                       ? S5_FLAG_AVOID_LEFT : S5_FLAG_AVOID_RIGHT);

            /* Width from the centre reading and the nearer side reading */
            float d_side = d_right;
            if (d_left > 0.0f && (d_right <= 0.0f || d_left < d_right)) {
                d_side = d_left;
            }
            float width = s5_estimate_width_cm(d_center, d_side, 45.0f);
            s5_publish_profile(dir, width, d_left, d_right);
        }

        tk_dly_tsk(100);
    }
}

EXPORT ER subsystem5_init(void)
{
    /* 1. Hardware driver initializations */
    ultrasonic_init();
    servo_init();

    /* 2. Create the event flag */
    T_CFLG cflg;
    cflg.flgatr = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = S5_FLAG_CLEAR;

    flg_s5_obstacle = tk_cre_flg(&cflg);
    if (flg_s5_obstacle < E_OK) {
        return (ER)flg_s5_obstacle;
    }

    return E_OK;
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