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
#define VC_CRUISE_CM_S      (20u)     // Normal driving speed (to be tuned).
#define VC_SLOW_CM_S        (10u)     // Speed while an obstacle is near.

// Mission state, 3 bits on the wire (8 values), one byte in RAM.
typedef uint8_t vc_state_t;
#define VC_STATE_IDLE       (0u)      // Waiting for START, motors off.
#define VC_STATE_FOLLOWING  (1u)      // Driving the course.
#define VC_STATE_TURNING    (2u)      // Reserved: barcode turn (stage 2).
#define VC_STATE_BYPASSING  (3u)      // Reserved: obstacle bypass (stage 3).
#define VC_STATE_BLOCKED    (4u)      // Stopped for an obstacle, resumes.
#define VC_STATE_STOPPED    (5u)      // Remote STOP; START resumes.
#define VC_STATE_FAULT      (6u)      // ESTOP or fault; RESET to IDLE.

IMPORT ER vehicle_init(void);
IMPORT ER vehicle_start(void);

/** Current mission state. Any task may read it (one byte, one writer). */
IMPORT vc_state_t vc_get_state(void);

#endif /* VEHICLE_H */

/*** end of file ***/
