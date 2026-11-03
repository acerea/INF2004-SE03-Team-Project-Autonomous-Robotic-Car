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

/* Event flag bits (Interface IF-06) */
#define S5_FLAG_CLEAR       0x01
#define S5_FLAG_SLOWDOWN    0x02
#define S5_FLAG_EMERGENCY   0x04
#define S5_FLAG_AVOID_LEFT  0x08
#define S5_FLAG_AVOID_RIGHT 0x10

#define S5_STOP_DIST_CM     15.0f
#define S5_WARN_DIST_CM     35.0f

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
