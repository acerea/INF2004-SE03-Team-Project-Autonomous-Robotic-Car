/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.
 */

#include "motion_control.h"
#include "subsystem2.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem2_test(void)
{
	T_RSEM	rsem;
	T_RMPF	rmpf;
	UW	seq;
	INT	v, bad = 0;

	TEST_INIT();

	/* S2-U01: every sample value is inside the documented 0..100 range */
	for (seq = 0; seq < 40; seq++) {
		v = s2_make_value(seq);
		if (v < 0 || v > 100) bad++;
	}
	TEST_CHECK("S2-U01", bad == 0, "sample values stay within 0..100");

	/* S2-U02: flow control never over- or under-counts */
	TEST_CHECK("S2-U02",
		   tk_ref_sem(sem_slots, &rsem) == E_OK &&
		   rsem.semcnt >= 0 && rsem.semcnt <= S2_POOL_COUNT &&
		   tk_ref_mpf(mpf_samples, &rmpf) == E_OK &&
		   rmpf.frbcnt <= S2_POOL_COUNT,
		   "free slots and free blocks within pool size");

		   /* 
		   Buddy 2: Basic motor movement test under micro T-kernel
		   */
		   // tm_printf((UB*)"[S2-M01] Starting motor test\n");

		   /* Allow time to connect the serial monitor */
		   // tk_dly_tsk(5000);

		   /* Move both wheels at 40% PWM */
		   // tm_printf((UB*)"[S2-M01] Moving forward now\n");
		   // tm_printf((UB*)"[S2-M01] Moving backward now\n");
		   // tm_printf((UB*)"[S2-M01] Moving left now\n");
		   // tm_printf((UB*)"[S2-M01] Moving right now\n");
		   
		   // motion_moveForward(40);
		   // motion_moveBackward(40);
		   // motion_turnLeft(40);
		   // motion_turnRight(40);

		   /* Move for 300 milliseconds */
		   // tk_dly_tsk(300);

		   /* Stop both motors */
		   // motion_stop();

		   // tm_printf((UB*)"[S2-M01] Motors stopped\n");

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
