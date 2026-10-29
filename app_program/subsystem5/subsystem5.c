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

LOCAL void s5_scan_task(INT stacd, void *param)
{
    ultrasonic_init();

    while (1) {
        float distance = ultrasonic_read_distance_cm();
        (void)distance;

        tk_dly_tsk(200); /* 5 Hz polling rate */
    }
}

EXPORT ER subsystem5_init(void)
{
    return E_OK;
}

EXPORT ER subsystem5_start(void)
{
    ID tid = app_start_task(s5_scan_task, S5_PRI_MAIN, 0, "s5_scan");
    return (tid < E_OK) ? tid : E_OK;
}
