/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5.  Built only with: make TEST=1
 *	Record results in testing/subsystem5/TEST_LOG.md.  IDs: S5-U01, ...
 */

#include "subsystem5.h"
#include "../test_report.h"
#include "drv_ultrasonic.h"

#if APP_TEST

EXPORT void subsystem5_test(void)
{
    TEST_INIT();

    /* S5-U01: Verify task context */
    TEST_CHECK("S5-U01", tk_get_tid() > 0, "running in a task context");

    /* S5-U02: Live sensor measurement check */
    ultrasonic_init();
    float dist = ultrasonic_read_distance_cm();
    TEST_CHECK("S5-U02", dist > 0.0f && dist < 400.0f, "ultrasonic returns valid range");

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
