/*
 * subsystem5.h
 * Subsystem 5 -- obstacle detection: public interface.
 *
 * Only what other subsystems use: the obstacle event flag and its bits,
 * the measured sizes for the bypass, and the sweep profile type.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"

#define S5_PRI_MAIN         20

/* Event Flag ID (defined and created in subsystem5.c) */
IMPORT ID flg_s5_obstacle;
/* Last estimated obstacle width in cm (0 = not measured). One writer:
   s5_scan_task. Read by the VC to size the bypass. */
IMPORT volatile UW g_s5_width_cm;
/* Front distance (cm) measured when stopping for the obstacle (0 = none). */
IMPORT volatile UW g_s5_front_cm;
/* How far (cm) the obstacle reaches past the line on the side chosen for
   the bypass, measured from the sensor's centre line (0 = at or short of
   the centre line). The VC goes this far sideways plus half the car and a
   margin. Written with the side flags, before AVOID_LEFT/RIGHT is set. */
IMPORT volatile UW g_s5_clear_cm;

/* Event flag bits (Interface IF-06) */
#define S5_FLAG_CLEAR       0x01
#define S5_FLAG_SLOWDOWN    0x02
#define S5_FLAG_EMERGENCY   0x04
#define S5_FLAG_AVOID_LEFT  0x08
#define S5_FLAG_AVOID_RIGHT 0x10

/* Only react to something within 15 cm (was 30 stop / 45 slow down).
 * WARN = STOP, so there is no slow-down band: clear, or stop at 15 cm. */
#define S5_STOP_DIST_CM     15.0f
#define S5_WARN_DIST_CM     15.0f

typedef enum {
    S5_DIR_NONE = 0,
    S5_DIR_LEFT,
    S5_DIR_RIGHT
} S5_BYPASS_DIR;

extern ID flg_s5_obstacle;

/* Obstacle profile from one servo sweep (s5_profile_analyse).
 * Angles: car frame, negative = LEFT, positive = RIGHT, S5_SWEEP_STEP_DEG
 * apart from -S5_SWEEP_MAX_DEG to +S5_SWEEP_MAX_DEG. */
#define S5_SWEEP_MAX_DEG    45      /* drv_servo's limit. */
#define S5_SWEEP_STEP_DEG   15
#define S5_SWEEP_N          ((2 * S5_SWEEP_MAX_DEG / S5_SWEEP_STEP_DEG) + 1)

typedef struct {
    float d_cm[S5_SWEEP_N];     /* In: reading per angle (cm, <= 0 = no echo). */
    float face_cm;              /* Out: distance to the obstacle's front face. */
    float left_cm;              /* Out: reach LEFT of the centre line (<0: stops short of it). */
    float right_cm;             /* Out: reach RIGHT of the centre line. */
    UB    left_open;            /* Out: 1 = still there at the end of the sweep (left_cm is a minimum). */
    UB    right_open;
    float width_cm;             /* Out: left_cm + right_cm (a minimum if a side is open). */
    S5_BYPASS_DIR dir;          /* Out: side to go round. */
    float clear_cm;             /* Out: reach on the chosen side, >= 0. */
} S5_PROFILE;

void s5_profile_analyse(S5_PROFILE *p, float d_stop_cm);

/* Core algorithm functions (older two-reading versions, kept for tests) */
S5_BYPASS_DIR s5_calculate_clearance(float d_left, float d_right);
float s5_estimate_width_cm(float d_center, float d_side, float angle_deg);

#endif /* SUBSYSTEM5_H */
