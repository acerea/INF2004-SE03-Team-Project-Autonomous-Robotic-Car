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
EXPORT volatile UW g_s5_width_cm = 0;
EXPORT volatile UW g_s5_front_cm = 0;

/* Forward prototypes */
static float fast_sqrtf(float val);

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
    /* -2 from the driver: the sensor did not answer at all. */
    proto_put(&bits, (d_center < -1.5f) ? PROTO_OBS_CM_FAULT
                                        : s5_cm_field(d_center, PROTO_OBS_CM_MAX),
              (UB)PROTO_OBS_CM_BITS);
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

/* Replace the old state bit with a new one (only one state bit is set).
 * Only on a change, and the new bit is set BEFORE the old one is cleared:
 * clearing first left a moment with no state bit at all, which the VC on
 * the other core reads as CLEAR (and so drove on). With both bits set for
 * a moment it sees the worse of the two. */
static UINT s5_state_bit = S5_FLAG_CLEAR;   /* = iflgptn at creation */

static void s5_set_state_flag(UINT flag)
{
    if (flag == s5_state_bit) {
        return;
    }
    tk_set_flg(flg_s5_obstacle, flag);
    tk_clr_flg(flg_s5_obstacle, ~(UINT)(S5_STATE_FLAGS & ~flag));
    s5_state_bit = flag;
}

/* Forget the last sweep. Called whenever the car is not stopped for an
 * obstacle, so a new EMERGENCY never starts with the old obstacle's side,
 * width or gap (the VC waits in BLOCKED until the new sweep sets them). */
static void s5_clear_side(void)
{
    tk_clr_flg(flg_s5_obstacle, ~(UINT)S5_SIDE_FLAGS);
    g_s5_width_cm = 0u;
    g_s5_front_cm = 0u;
}

/* Point the servo and wait for it to settle, but only if it has to move:
 * the old loop waited 150 ms every time, even when already centred. */
static INT s5_servo_angle = 1000;           /* unknown at start-up */

#define S5_PAN_STEP_DEG   5       /* Servo moves this much per step... */
#define S5_PAN_STEP_MS    25      /* ...then waits this long (~200 deg/s). */
#define S5_PAN_SETTLE_MS  80      /* Final settle before a reading. */

/* Pan gently: step from the current angle to the target instead of
 * snapping, so the sensor head does not jerk the car about. */
static void s5_point(INT angle)
{
    if (angle == s5_servo_angle) {
        return;
    }
    if (1000 == s5_servo_angle) {               /* First use: position unknown. */
        servo_set_angle(angle);
        s5_servo_angle = angle;
        tk_dly_tsk(150);
        return;
    }
    while (s5_servo_angle != angle) {
        INT diff = angle - s5_servo_angle;
        if (diff > S5_PAN_STEP_DEG)        diff = S5_PAN_STEP_DEG;
        else if (diff < -S5_PAN_STEP_DEG)  diff = -S5_PAN_STEP_DEG;
        s5_servo_angle += diff;
        servo_set_angle(s5_servo_angle);
        tk_dly_tsk(S5_PAN_STEP_MS);
    }
    tk_dly_tsk(S5_PAN_SETTLE_MS);
}

#define S5_SCAN_NEAR_DEG    20      /* Second, squarer look at each side. */
#define S5_SIDE_BLOCKED_CM  60.0f   /* Side obstacle nearer than this: blocked. */

/* Look at one side at two angles. Returns the nearest valid distance (cm)
 * or a negative value if neither angle got an echo; *p_far gets the far
 * (45 deg) reading for the width estimate. */
static float s5_scan_side(INT near_deg, INT far_deg, float *p_far)
{
    s5_point(near_deg);
    float d_near = ultrasonic_read_distance_filtered();
    tk_dly_tsk(60);

    s5_point(far_deg);
    float d_far = ultrasonic_read_distance_filtered();
    tk_dly_tsk(60);

    *p_far = d_far;
    if (d_near > 0.0f && (d_far <= 0.0f || d_near < d_far)) {
        return d_near;
    }
    return d_far;
}

/* Pick the bypass side. A side with an obstacle within S5_SIDE_BLOCKED_CM is
 * blocked; go to the other side. Both blocked: the one with more room.
 * Neither blocked: a side with no echo is open, else the one with more room
 * (default right). */
static S5_BYPASS_DIR s5_choose_side(float d_left, float d_right)
{
    INT left_blocked  = (d_left  > 0.0f) && (d_left  < S5_SIDE_BLOCKED_CM);
    INT right_blocked = (d_right > 0.0f) && (d_right < S5_SIDE_BLOCKED_CM);

    if (left_blocked && !right_blocked) return S5_DIR_RIGHT;
    if (right_blocked && !left_blocked) return S5_DIR_LEFT;
    if (left_blocked && right_blocked) {
        return (d_left > d_right) ? S5_DIR_LEFT : S5_DIR_RIGHT;
    }
    if (d_left <= 0.0f && d_right > 0.0f) return S5_DIR_LEFT;
    if (d_right <= 0.0f && d_left > 0.0f) return S5_DIR_RIGHT;
    if (d_left > 0.0f && d_right > 0.0f) {
        return (d_left > d_right) ? S5_DIR_LEFT : S5_DIR_RIGHT;
    }
    return S5_DIR_RIGHT;
}

#define S5_RELEASE_MARGIN_CM  5.0f  /* Must read this far past the stop
                                         distance to release EMERGENCY. */
#define S5_RELEASE_READINGS   3u    /* ...this many times in a row. */

static UW s5_in_emergency = 0u;     /* Only s5_scan_task uses these. */
static UW s5_release_count = 0u;
static UW s5_fault_reported = 0u;

LOCAL void s5_scan_task(INT stacd, void *param)
{
    (void)stacd;
    (void)param;

    ultrasonic_init();
    servo_init();

    while (1) {
        /* 1. Normal forward observation */
        s5_point(SERVO_ANGLE_CENTER);

        /* Read forward distance: median of 3 pings, so one bad ping cannot
           flip the state. */
        float d_center = ultrasonic_read_distance_filtered();
        tk_dly_tsk(60);  /* Ultrasonic acoustic decay holdoff */

        /* 2. Decide the state first, so EMERGENCY is not delayed by the
              sweep below, and publish it every loop (also on no echo). */
        UW state = S5_TLM_CLEAR;

        /* Hysteresis: once stopped for an obstacle, stay in EMERGENCY until
           S5_RELEASE_READINGS readings in a row are clearly beyond the stop distance. A
           dropout (no echo) or a reading just past the limit does not count,
           so the car cannot creep forward into the obstacle. */
        float d_raw = d_center;     /* What the sensor really read. */

        /* Sensor not answering at all (-2): this used to fall through to
           CLEAR, so the car drove on blind. Stop instead, with no side
           bits, so the VC waits in BLOCKED and never bypasses blind. */
        if (d_center < -1.5f) {
            s5_set_state_flag(S5_FLAG_EMERGENCY);
            s5_clear_side();
            s5_in_emergency = 1u;
            s5_release_count = 0u;
            s5_publish_obstacle(S5_TLM_EMERGENCY, d_raw);
            if (!s5_fault_reported) {
                tm_printf((UB*)"s5: ultrasonic not answering, holding EMERGENCY\n");
                s5_fault_reported = 1u;
            }
            tk_dly_tsk(100);
            continue;
        }
        s5_fault_reported = 0u;

        if (s5_in_emergency) {
            /* Clearly further away, or no echo at all (open air; not
               "disconnected", which stays stopped to be safe). */
            if ((d_center > (S5_STOP_DIST_CM + S5_RELEASE_MARGIN_CM))
                || ((d_center < 0.0f) && (d_center > -1.5f))) {
                s5_release_count++;
            } else {
                s5_release_count = 0u;
            }
            if (s5_release_count < S5_RELEASE_READINGS) {
                d_center = (d_center > 0.0f && d_center < S5_STOP_DIST_CM)
                           ? d_center : (S5_STOP_DIST_CM - 0.1f);
            }
        }

        if (d_center > 0.0f && d_center < S5_STOP_DIST_CM) {
            state = S5_TLM_EMERGENCY;
            s5_set_state_flag(S5_FLAG_EMERGENCY);
        } else if (d_center > 0.0f && d_center < S5_WARN_DIST_CM) {
            state = S5_TLM_SLOWDOWN;
            s5_set_state_flag(S5_FLAG_SLOWDOWN);
            s5_clear_side();
        } else {
            s5_set_state_flag(S5_FLAG_CLEAR);
            s5_clear_side();
        }
        s5_in_emergency = (state == S5_TLM_EMERGENCY) ? 1u : 0u;
        if (!s5_in_emergency) {
            s5_release_count = 0u;
        }
        /* Show the real reading (e.g. disconnected), even while held. */
        s5_publish_obstacle(state, d_raw);

        /* Sweep the sides only when stopped (EMERGENCY): in SLOWDOWN the car
           is still moving and the sensor must keep looking ahead. */
        if (state == S5_TLM_EMERGENCY) {
            /* 3-4. Sweep each side at 20 and 45 deg, median of 3 pings at
             *      each. A flat box at 45 deg often reflects the sound away
             *      (no echo); at 20 deg it is hit more squarely, so an
             *      obstacle on that side is actually seen. */
            float d_left45 = 0.0f;
            float d_right45 = 0.0f;
            float d_left = s5_scan_side(-S5_SCAN_NEAR_DEG, SERVO_ANGLE_LEFT,
                                        &d_left45);
            float d_right = s5_scan_side(S5_SCAN_NEAR_DEG, SERVO_ANGLE_RIGHT,
                                         &d_right45);

            /* Return to center and settle */
            s5_point(SERVO_ANGLE_CENTER);

            /* 5. Choose direction, replace the old side bit, publish */
            S5_BYPASS_DIR dir = s5_choose_side(d_left, d_right);
            tk_clr_flg(flg_s5_obstacle, ~(UINT)S5_SIDE_FLAGS);
            tk_set_flg(flg_s5_obstacle, (dir == S5_DIR_LEFT)
                       ? S5_FLAG_AVOID_LEFT : S5_FLAG_AVOID_RIGHT);

            /* Width from the centre reading and the nearer 45 deg reading
               (the width formula assumes the 45 deg angle). */
            float d_side = d_right45;
            if (d_left45 > 0.0f && (d_right45 <= 0.0f || d_left45 < d_right45)) {
                d_side = d_left45;
            }
            float width = s5_estimate_width_cm(d_center, d_side, 45.0f);
            g_s5_width_cm = (width > 0.0f) ? (UW)(width + 0.5f) : 0u;
            g_s5_front_cm = (d_center > 0.0f) ? (UW)(d_center + 0.5f) : 0u;
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
    T_CFLG cflg = {0};
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