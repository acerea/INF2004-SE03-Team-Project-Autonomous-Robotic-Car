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
EXPORT volatile UW g_s5_clear_cm = 0;

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

/* left/right: how far the obstacle reaches each side of the centre line
 * (cm, 0-400), S5_EXTENT_OPEN (511) = past the end of the sweep. */
static void
s5_publish_profile (S5_BYPASS_DIR dir, float width_cm, UW left_field,
                    UW right_field)
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
    proto_put(&bits, left_field, (UB)PROTO_PRO_LEFT_BITS);
    proto_put(&bits, right_field, (UB)PROTO_PRO_RIGHT_BITS);
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
    g_s5_clear_cm = 0u;
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

/* ---- Obstacle profile sweep -------------------------------------------
 * Old way (log 25 Nov 18:26): one reading at 20 and 45 deg each side, then
 * "go to the side with the bigger reading". A box at 15 cm is hit by BOTH
 * 20 deg beams (14 vs 13 cm), so 1 cm of noise picked the side, and the
 * width used the law of cosines, which gives the distance between two
 * points on the box, not its width.
 * Now: read every S5_SWEEP_STEP_DEG across the front. A reading belongs to
 * the obstacle if it is about as far ahead as the front face (forward
 * part d*cos(a) <= face + S5_FACE_TOL_CM). Walking out from the centre on
 * each side, the obstacle's edge is between the last hit and the first
 * miss; it reaches face*tan(edge) sideways. Go round the side it reaches
 * LESS far: an obstacle sitting more to the left is passed on the right. */

/* +1: a positive servo angle looks to the car's RIGHT (drv_servo.h names
 * -45 "LEFT"). If the bench check shows a box on the left reported on the
 * right, set -1. */
#define S5_SERVO_RIGHT_SIGN   (1)
#define S5_FACE_TOL_CM        8.0f  /* Deeper than face + this: past the obstacle. */
/* The ultrasonic beam is not a thin ray: it still echoes off the box a few
 * degrees past the edge, so each edge is pulled in by this much.
 * CALIBRATE: put the demo obstacle 15 cm ahead, read "[OBS] ... width"
 * and raise this if the width is too big, lower it if too small. */
#define S5_BEAM_HALF_DEG      12.0f
#define S5_SIDE_TIE_CM        2.0f  /* Reaches within this on both sides: go right. */
#define S5_EXTENT_OPEN        511u  /* Telemetry: past the end of the sweep. */
#define S5_EXTENT_MAX         400u

#define S5_DEG_TO_RAD         0.017453293f

/* tan and cos for |deg| <= 50 (series; 0.5 % at 45 deg, no libm). */
static float s5_tan_deg(float deg)
{
    float x  = deg * S5_DEG_TO_RAD;
    float x2 = x * x;
    return x * (1.0f + x2 * (0.33333333f + x2 * (0.13333333f + x2 * 0.05396825f)));
}

static float s5_cos_deg(float deg)
{
    float x2 = (deg * S5_DEG_TO_RAD) * (deg * S5_DEG_TO_RAD);
    return 1.0f - x2 * (0.5f - x2 * (0.04166667f - x2 * 0.00138889f));
}

static S5_PROFILE s5_prof;          /* Static: s5_scan_task's stack is small. */
static UW s5_swept = 0u;            /* One sweep per stop (see s5_scan_task). */

static void s5_print_profile(const S5_PROFILE *p)
{
    tm_printf((UB*)"[OBS] sweep mm (left..right):");
    for (INT i = 0; i < S5_SWEEP_N; i++) {
        INT a = -S5_SWEEP_MAX_DEG + (i * S5_SWEEP_STEP_DEG);
        if (p->d_cm[i] > 0.0f) {
            tm_printf((UB*)" %d:%d", a, (INT)(p->d_cm[i] * 10.0f));
        } else {
            tm_printf((UB*)" %d:-", a);
        }
    }
    tm_printf((UB*)"\n[OBS] face %d mm, reaches left %d%s mm, right %d%s mm,"
              " width %d%s mm -> go %s, clear %d mm\n",
              (INT)(p->face_cm * 10.0f),
              (INT)(p->left_cm * 10.0f),  p->left_open ? "+" : "",
              (INT)(p->right_cm * 10.0f), p->right_open ? "+" : "",
              (INT)(p->width_cm * 10.0f),
              (p->left_open || p->right_open) ? "+" : "",
              (S5_DIR_LEFT == p->dir) ? "LEFT" : "RIGHT",
              (INT)(p->clear_cm * 10.0f));
}

/* Sweep left to right, one median-of-3 reading per angle (~3 s). */
static void s5_sweep(S5_PROFILE *p)
{
    for (INT i = 0; i < S5_SWEEP_N; i++) {
        INT a = -S5_SWEEP_MAX_DEG + (i * S5_SWEEP_STEP_DEG);
        s5_point(a * S5_SERVO_RIGHT_SIGN);
        p->d_cm[i] = ultrasonic_read_distance_filtered();
        tk_dly_tsk(60);             /* Acoustic decay before the next one. */
    }
    s5_point(SERVO_ANGLE_CENTER);
}

static UW s5_extent_field(float cm, UB open)
{
    if (open) {
        return S5_EXTENT_OPEN;
    }
    if (cm <= 0.0f) {
        return 0u;
    }
    return (cm >= (float)S5_EXTENT_MAX) ? S5_EXTENT_MAX : (UW)(cm + 0.5f);
}

#define S5_RELEASE_MARGIN_CM  5.0f  /* Must read this far past the stop
                                         distance to release EMERGENCY. */
#define S5_RELEASE_READINGS   3u    /* ...this many times in a row. */

static UW s5_in_emergency = 0u;     /* Only s5_scan_task uses these. */
static UW s5_release_count = 0u;
static UW s5_fault_reported = 0u;

/* One bad reading must not stop the car (log 25 Nov 17:25-17:26: stops
 * at 4-5 cm and one "not answering" with nothing in front). Each reading
 * is already a median of 3 pings; these need several in a row. */
#define S5_NEAR_READINGS      2u    /* < stop distance, to enter EMERGENCY. */
#define S5_FAULT_READINGS     3u    /* No answer at all, to hold EMERGENCY. */
static UW s5_near_run = 0u;
static UW s5_fault_run = 0u;

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
        if ((d_center < -1.5f) && (s5_fault_run < S5_FAULT_READINGS)) {
            s5_fault_run++;
        }
        if ((d_center < -1.5f) && (s5_fault_run < S5_FAULT_READINGS)) {
            tk_dly_tsk(100);        /* Maybe a glitch: keep the old state. */
            continue;
        }
        if (d_center >= -1.5f) {
            s5_fault_run = 0u;
        }
        if (d_center < -1.5f) {
            s5_set_state_flag(S5_FLAG_EMERGENCY);
            s5_clear_side();
            s5_swept = 0u;              /* Sweep again once it answers. */
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

        /* Not stopped yet: a near reading only counts once it repeats. */
        if (!s5_in_emergency && (d_center > 0.0f)
            && (d_center < S5_STOP_DIST_CM)) {
            if (s5_near_run < S5_NEAR_READINGS) {
                s5_near_run++;
            }
            if (s5_near_run < S5_NEAR_READINGS) {
                d_center = -1.0f;   /* Treat as clear for this reading. */
            }
        } else {
            s5_near_run = 0u;
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

        /* Sweep only when stopped (EMERGENCY), and only ONCE per stop:
           the old code swept again every loop while the car was already
           turning away (log 18:26:19: a second "width 53 cm" mid-bypass),
           and the servo panning while driving shakes the readings. */
        if (state != S5_TLM_EMERGENCY) {
            s5_swept = 0u;
        } else if (!s5_swept) {
            s5_sweep(&s5_prof);
            s5_profile_analyse(&s5_prof, d_raw);
            s5_print_profile(&s5_prof);

            /* Sizes first, then the side bit: the VC starts the bypass the
               moment it sees a side bit and reads the sizes then. */
            g_s5_width_cm = (s5_prof.width_cm > 0.0f)
                            ? (UW)(s5_prof.width_cm + 0.5f) : 0u;
            g_s5_front_cm = (s5_prof.face_cm > 0.0f)
                            ? (UW)(s5_prof.face_cm + 0.5f) : 0u;
            g_s5_clear_cm = (UW)(s5_prof.clear_cm + 0.5f);
            tk_clr_flg(flg_s5_obstacle, ~(UINT)S5_SIDE_FLAGS);
            tk_set_flg(flg_s5_obstacle, (s5_prof.dir == S5_DIR_LEFT)
                       ? S5_FLAG_AVOID_LEFT : S5_FLAG_AVOID_RIGHT);

            s5_publish_profile(s5_prof.dir, s5_prof.width_cm,
                               s5_extent_field(s5_prof.left_cm, s5_prof.left_open),
                               s5_extent_field(s5_prof.right_cm, s5_prof.right_open));
            s5_swept = 1u;
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

/* Edge of the obstacle on one side, from the sweep. step = +1 walks right,
 * -1 walks left. Returns how far it reaches that side (cm, + = away from
 * the centre line); *p_open = 1 if it was still there at the last angle. */
static float s5_side_reach(const S5_PROFILE *p, INT step, UB *p_open)
{
    INT c = S5_SWEEP_N / 2;
    INT last = c;                   /* The centre beam hit it: that's why we stopped. */
    INT i;

    *p_open = 0u;
    for (i = c + step; (i >= 0) && (i < S5_SWEEP_N); i += step) {
        float a = (float)((i - c) * S5_SWEEP_STEP_DEG);
        float d = p->d_cm[i];
        if (d <= 0.0f) {
            continue;               /* No echo: a flat face at a steep angle
                                       often sends the sound away. Not proof
                                       the obstacle ended; look further. */
        }
        if ((d * s5_cos_deg(a)) <= (p->face_cm + S5_FACE_TOL_CM)) {
            last = i;               /* About as far ahead as the face: on it. */
        } else {
            break;                  /* Echo from further back: past the edge. */
        }
    }
    float a_last = (float)((last - c) * S5_SWEEP_STEP_DEG);
    if (a_last < 0.0f) {
        a_last = -a_last;
    }
    if (last == (c + (step * c))) {
        /* Still on it at the last angle. The beam may only be catching
           the corner at its own edge, so all we know is that it reaches
           at least this far: a minimum. */
        *p_open = 1u;
        return p->face_cm * s5_tan_deg(a_last - S5_BEAM_HALF_DEG);
    }
    /* Edge half-way to the first miss, less the beam's spread. */
    float edge = a_last + (0.5f * (float)S5_SWEEP_STEP_DEG) - S5_BEAM_HALF_DEG;
    return p->face_cm * s5_tan_deg(edge);
}

/* Work out the obstacle's reach each side, its width and the side to go
 * round. d_stop_cm: the reading that stopped the car (used if the centre
 * beam of the sweep got no echo). Pure function: unit-tested. */
void s5_profile_analyse(S5_PROFILE *p, float d_stop_cm)
{
    INT c = S5_SWEEP_N / 2;

    p->face_cm = (p->d_cm[c] > 0.0f) ? p->d_cm[c] : d_stop_cm;
    if (p->face_cm <= 0.0f) {
        p->face_cm = S5_STOP_DIST_CM;
    }
    p->left_cm  = s5_side_reach(p, -1, &p->left_open);
    p->right_cm = s5_side_reach(p, 1, &p->right_open);
    p->width_cm = p->left_cm + p->right_cm;
    if (p->width_cm < 0.0f) {
        p->width_cm = 0.0f;
    }

    /* Go round the side it reaches less far. An open side (still there
     * at 40 deg) is a last resort. Nearly equal: right (the default). */
    if (p->left_open && !p->right_open) {
        p->dir = S5_DIR_RIGHT;
    } else if (p->right_open && !p->left_open) {
        p->dir = S5_DIR_LEFT;
    } else if (p->left_cm < (p->right_cm - S5_SIDE_TIE_CM)) {
        p->dir = S5_DIR_LEFT;
    } else {
        p->dir = S5_DIR_RIGHT;
    }

    p->clear_cm = (S5_DIR_LEFT == p->dir) ? p->left_cm : p->right_cm;
    if (p->clear_cm < 0.0f) {
        p->clear_cm = 0.0f;
    }
}
