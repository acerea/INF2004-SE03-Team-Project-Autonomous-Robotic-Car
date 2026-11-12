/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem3/TEST_LOG.md.
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

LOCAL APP_STATS read_stats(void)
{
	APP_STATS	s;

	tk_loc_mtx(mtx_stats, TMO_FEVR);
	s = app_stats;
	tk_unl_mtx(mtx_stats);
	return s;
}

EXPORT void subsystem3_test(void)
{
	APP_STATS	stats;

	TEST_INIT();

	stats = read_stats();

	/* S3-U01: an empty statistics set is valid before the first sample. */
	TEST_CHECK("S3-U01",
		   stats.received == 0 ||
		   (stats.min <= stats.max &&
		    stats.min >= 0 && stats.max <= 100),
		   "empty or min <= max with values within 0..100");

	TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
