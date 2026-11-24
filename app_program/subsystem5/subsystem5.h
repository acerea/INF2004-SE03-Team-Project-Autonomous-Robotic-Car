/*
 *	subsystem5.h
 *	Subsystem 5 -- <name>.  Student ID: <student ID>.
 *
 *	EMPTY TEMPLATE.  Subsystems 1-4 show a complete example; copy their
 *	pattern.  Put here only what OTHER subsystems may use (object IDs,
 *	message formats, priorities); keep everything else LOCAL in the .c.
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

/* Core algorithm functions */
S5_BYPASS_DIR s5_calculate_clearance(float d_left, float d_right);
float s5_estimate_width_cm(float d_center, float d_side, float angle_deg);

#endif /* SUBSYSTEM5_H */
