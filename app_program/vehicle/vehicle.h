/** @file vehicle.h
 *
 * @brief Vehicle Controller (VC): public interface.
 *
 * @par
 * The VC holds the mission state and is the only task that sends motion
 * requests to subsystem 2. Other subsystems report to it through their own
 * kernel objects (S1: g_flg_remote/g_mbf_remote_cmd, S5: flg_s5_obstacle).
 * Owner: Buddy 1 (2502049), integration role.
 */
#ifndef VEHICLE_H
#define VEHICLE_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"

#define VC_PRI              (10)      // Highest application priority.
#define VC_TICK_MS          (10u)     // Input polling period.
#define VC_CRUISE_CM_S      (3u)      // Default cruise: 30 mm/s (MODE changes it).
#define VC_SLOW_CM_S        (5u)      // Speed while an obstacle is near.

/* Conservative, encoder-driven obstacle bypass profile.  S2 interprets
 * turn arguments as degrees and distance arguments as centimetres. */
#define VC_BYPASS_TURN_DEG          (90u)
#define VC_BYPASS_LATERAL_CM         (25u)
#define VC_BYPASS_FORWARD_CM         (60u)
#define VC_CAR_HALF_WIDTH_CM         (8u)    // Half the car body width.
#define VC_BYPASS_MARGIN_CM          (5u)    // Extra room beside the obstacle.
#define VC_BYPASS_LATERAL_MIN_CM     (15u)
#define VC_BYPASS_LATERAL_MAX_CM     (40u)
#define VC_OBSTACLE_DEPTH_CM         (15u)   // Obstacle front-to-back: measure the demo one.
#define VC_CAR_HALF_LENGTH_CM        (10u)   // Half the car length.
#define VC_BYPASS_FORWARD_MIN_CM     (30u)
#define VC_BYPASS_FORWARD_MAX_CM     (120u)
#define VC_BYPASS_TIMEOUT_MS         (30000u)
#define VC_REJOIN_TIMEOUT_MS         (3000u)
#define VC_REJOIN_STABLE_SAMPLES     (3u)

// Mission state, 3 bits on the wire (8 values), one byte in RAM.
typedef uint8_t vc_state_t;
#define VC_STATE_IDLE       (0u)      // Waiting for START, motors off.
#define VC_STATE_FOLLOWING  (1u)      // Driving the course.
#define VC_STATE_TURNING     (2u)      // Barcode turn (reserved for navigation).
#define VC_STATE_BYPASSING   (3u)      // Encoder/timed obstacle bypass.
#define VC_STATE_BLOCKED    (4u)      // Stopped for an obstacle, resumes.
#define VC_STATE_STOPPED    (5u)      // Remote STOP; START resumes.
#define VC_STATE_FAULT      (6u)      // ESTOP or fault; RESET to IDLE.
#define VC_STATE_CALIBRATING (7u)     // Sweep IR calibration (turns on the spot).

/* Sweep calibration: left by this angle, right by twice it, back to the
 * start heading. Every IR sensor must cross the line and the white floor
 * on each side of it, so the angle depends on how far ahead of the wheels
 * the sensors sit: increase it if a sensor reports "low contrast". */
#define VC_CAL_SWEEP_DEG        (40u)
#define VC_CAL_STEP_TIMEOUT_MS  (6000u)

/* After the sweep: turn towards the line until the middle sensor is on it,
 * up to VC_CENTRE_DEG one way, then twice that back the other way. */
#define VC_CENTRE_DEG           (30u)
#define VC_CENTRE_WAIT_MS       (1500u)  // For IR to apply the new levels.
#define VC_CENTRE_MAX_MS        (20000u) // Give up centring after this.

/* Nudges (line search and centring): turn VC_NUDGE_DEG, stop, wait
 * VC_NUDGE_SETTLE_MS, look. Smaller = more precise but slower. */
#define VC_NUDGE_DEG            (6u)
#define VC_NUDGE_SETTLE_MS      (150u)
#define VC_NUDGE_TURN_MAX_MS    (2000u)  // One nudge never takes longer.
#define VC_NUDGE_FIX_STEPS      (4u)     // Nudges back after an overshoot.
#define VC_NUDGE_FIXES_MAX      (3u)

/* Line search after the IR line-lost timeout: turn towards the side the
 * line was last seen, then back past the start the other way. */
#define VC_SEARCH_DEG           (45u)

/* Line lost while following: drive straight this long before searching
 * (counted from when the VC itself lost it, not from before START). */
#define VC_LOST_STRAIGHT_MS     (1200u)

IMPORT ER vehicle_init(void);
IMPORT ER vehicle_start(void);

/** Current mission state. Any task may read it (one byte, one writer). */
IMPORT vc_state_t vc_get_state(void);

#endif /* VEHICLE_H */

/*** end of file ***/