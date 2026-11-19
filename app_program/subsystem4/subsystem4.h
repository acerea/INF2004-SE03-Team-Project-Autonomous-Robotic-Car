/*
 *	subsystem4.h
 *	Subsystem 4 -- IMU and motion sensing.  Student ID: 2503813.
 *
 *	Public interface of subsystem 4.  Other subsystems include it as
 *	"../subsystem4/subsystem4.h".
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

#define S4_PRI_MONITOR		18	/* priority band 18-19 (TEAM.md) */
#define S4_PRI_IMU		19	/* GY-511 sampling task; same S4 priority band */
#define S4_ALARM_MS		2000	/* report anyway if nothing for 2 s */
#define S4_LED_TOGGLE_REPORTS	6	/* pause/resume the LED every N reports */

/* IF-03: send one text line (at most S4_SUMMARY_MAX bytes, no '\0'
   needed) to mbf_summary, then set FLG_BATCH in flg_events. */
#define S4_SUMMARY_MAX		48
#define FLG_BATCH		0x0002	/* set by subsystem 3 */
#define FLG_ALARM		0x0001	/* private: set by our alarm handler */

IMPORT ID	mbf_summary;
IMPORT ID	flg_events;

/* IF-13: S4 -> VC.  Event flag owned by S4, written ONLY by b4_imu_task.
   The bits are LEVELS: read them with tk_ref_flg, never clear them.
   The VC stops the motors when LIFTED or OVERTURNED is set
   (change of requirement: collision detection replaced by lift/overturn). */
IMPORT ID	flg_s4_imu;
#define S4_FLG_LIFTED		0x01u	/* robot picked up: end of run      */
#define S4_FLG_OVERTURNED	0x02u	/* robot on its side or upside down */
#define S4_FLG_IMU_FAULT	0x04u	/* IMU absent or not answering      */
#define S4_FLG_ALL		(S4_FLG_LIFTED | S4_FLG_OVERTURNED | \
				 S4_FLG_IMU_FAULT)

/* Private to subsystem 4 */
IMPORT ID	alm_monitor;
IMPORT UW	s4_lines_received;	/* for the tests */
IMPORT void	monitor_task(INT stacd, void *exinf);
IMPORT void	monitor_alarm_handler(void *exinf);
IMPORT void	b4_imu_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM4_H */
