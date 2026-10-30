/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem4/TEST_LOG.md and the
 *	INT- result in testing/integration/TEST_LOG.md.
 */

#include "buddy4_data.h"
#include "buddy4_classifier.h"
#include "buddy4_ramp.h"
#include "subsystem4.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem4_test(void)
{
	T_RMBF rmbf;
	UW before;
	B4_MOTION_INPUT input;
	B4_EVENT classified;
	B4_RAMP_RESULT ramp;

	TEST_INIT();

	/* B4-U01: state and flags fit into one byte. */
	B4_EVENT event = b4_pack_event(
		B4_STATE_TURNING, 1u, 0u);

	TEST_CHECK("B4-U01",
			   event == 0x0Bu,
			   "turning state plus collision flag packs correctly");

	/* B4-U02: state can be decoded. */
	TEST_CHECK("B4-U02",
			   b4_event_state(event) == B4_STATE_TURNING,
			   "motion state can be extracted");

	/* B4-U03: collision and IMU fault flags decode correctly. */
	TEST_CHECK("B4-U03",
			   b4_event_has_collision(event) &&
				   !b4_event_has_imu_fault(event),
			   "collision and IMU fault flags are independent");

	/* B4-U04: invalid states are converted to UNKNOWN. */
	event = b4_pack_event(8u, 0u, 0u);

	TEST_CHECK("B4-U04",
			   b4_event_state(event) == B4_STATE_UNKNOWN,
			   "invalid state code is handled safely");

	/* S4-U01: the message buffer accepts the documented line length */
	TEST_CHECK("S4-U01",
			   tk_ref_mbf(mbf_summary, &rmbf) == E_OK &&
				   rmbf.maxmsz == S4_SUMMARY_MAX,
			   "message buffer max line = S4_SUMMARY_MAX");

	/* INT-02 (integration, IF-03): summary lines from subsystem 3 reach
	   the monitor.  One line per 5 samples = one per 1.5 s. */
	before = s4_lines_received;
	tk_dly_tsk(3500);
	TEST_CHECK("INT-02", s4_lines_received >= before + 2,
			   "summary lines flow from subsystem 3 to subsystem 4");

	/* Stage 2: test the sensor-independent motion classifier. */

	/* B4-U05: Low activity should classify as stationary. */
	input.motion_level_mg = 0;
	input.vertical_delta_mg = 0;
	input.yaw_rate_mdps = 0;
	input.imu_fault = 0u;

	classified = b4_classify_motion(input);

	TEST_CHECK("B4-U05",
			   b4_event_state(classified) == B4_STATE_STATIONARY,
			   "low activity is classified as stationary");

	/* B4-U06: Sufficient activity should classify as moving. */
	input.motion_level_mg = 250;
	input.vertical_delta_mg = 0;
	input.yaw_rate_mdps = 0;
	input.imu_fault = 0u;

	classified = b4_classify_motion(input);

	TEST_CHECK("B4-U06",
			   b4_event_state(classified) == B4_STATE_MOVING,
			   "sufficient activity is classified as moving");

	/* B4-U07: A valid int16 yaw-rate test value at the threshold classifies as turning. */
	input.motion_level_mg = 100;
	input.vertical_delta_mg = 0;
	input.yaw_rate_mdps = 30000;
	input.imu_fault = 0u;

	classified = b4_classify_motion(input);

	TEST_CHECK("B4-U07",
			   b4_event_state(classified) == B4_STATE_TURNING,
			   "high yaw rate is classified as turning");

	/* B4-U08: Positive vertical deviation suggests climbing. */
	input.motion_level_mg = 100;
	input.vertical_delta_mg = 900;
	input.yaw_rate_mdps = 0;
	input.imu_fault = 0u;

	classified = b4_classify_motion(input);

	TEST_CHECK("B4-U08",
			   b4_event_state(classified) == B4_STATE_CLIMBING,
			   "positive vertical deviation is classified as climbing");

	/* B4-U09: Negative vertical deviation suggests descending. */
	input.motion_level_mg = 100;
	input.vertical_delta_mg = -900;
	input.yaw_rate_mdps = 0;
	input.imu_fault = 0u;

	classified = b4_classify_motion(input);

	TEST_CHECK("B4-U09",
			   b4_event_state(classified) == B4_STATE_DESCENDING,
			   "negative vertical deviation is classified as descending");

	/* B4-U10: Large vertical deviation indicates a possible impact. */
	input.motion_level_mg = 100;
	input.vertical_delta_mg = 2000;
	input.yaw_rate_mdps = 0;
	input.imu_fault = 0u;

	classified = b4_classify_motion(input);

	TEST_CHECK("B4-U10",
			   b4_event_state(classified) == B4_STATE_IMPACT &&
				   b4_event_has_collision(classified),
			   "large vertical deviation sets impact and collision");

	/* B4-U11: An IMU fault should produce a fault event. */
	input.motion_level_mg = 0;
	input.vertical_delta_mg = 0;
	input.yaw_rate_mdps = 0;
	input.imu_fault = 1u;

	classified = b4_classify_motion(input);

	TEST_CHECK("B4-U11",
			   b4_event_state(classified) == B4_STATE_FAULT &&
				   b4_event_has_imu_fault(classified),
			   "IMU fault produces a fault event");

	/* B4-U12: Motion just below the threshold should be stationary. */
	input.motion_level_mg = 149;
	input.vertical_delta_mg = 0;
	input.yaw_rate_mdps = 0;
	input.imu_fault = 0u;

	classified = b4_classify_motion(input);

	TEST_CHECK("B4-U12",
			   b4_event_state(classified) == B4_STATE_STATIONARY,
			   "activity just below threshold is stationary");

	/* B4-U13: Motion at the threshold should be moving. */
	input.motion_level_mg = 150;
	input.vertical_delta_mg = 0;
	input.yaw_rate_mdps = 0;
	input.imu_fault = 0u;

	classified = b4_classify_motion(input);

	TEST_CHECK("B4-U13",
			   b4_event_state(classified) == B4_STATE_MOVING,
			   "activity at threshold is classified as moving");

	/* B4-U14: Negative yaw rate should also detect turning. */
	input.motion_level_mg = 100;
	input.vertical_delta_mg = 0;
	input.yaw_rate_mdps = -30000;
	input.imu_fault = 0u;

	classified = b4_classify_motion(input);

	TEST_CHECK("B4-U14",
			   b4_event_state(classified) == B4_STATE_TURNING,
			   "large negative yaw rate is classified as turning");

	/* B4-U15: An IMU fault takes priority over an impact condition. */
	input.motion_level_mg = 100;
	input.vertical_delta_mg = 2000;
	input.yaw_rate_mdps = 0;
	input.imu_fault = 1u;

	classified = b4_classify_motion(input);

	TEST_CHECK("B4-U15",
			   b4_event_state(classified) == B4_STATE_FAULT &&
				   b4_event_has_imu_fault(classified) &&
				   !b4_event_has_collision(classified),
			   "IMU fault takes priority over impact classification");

	/* Ramp-estimator tests use synthetic inputs; these do not test the IMU. */
	/* B4-R01: a 100 mm ramp distance at 30 degrees gives about 50 mm rise. */
	ramp = b4_estimate_ramp_height(100u, 300u);
	TEST_CHECK("B4-R01",
			   ramp.valid && ramp.height_mm >= 49u && ramp.height_mm <= 51u,
			   "30 degree ramp height calculation is approximately correct");

	/* B4-R02: zero angle is rejected as an invalid ramp estimate. */
	ramp = b4_estimate_ramp_height(100u, 0u);
	TEST_CHECK("B4-R02",
			   !ramp.valid && ramp.height_mm == B4_RAMP_INVALID_HEIGHT,
			   "zero ramp angle returns invalid result");

	/* B4-R03: an angle above 90 degrees is rejected. */
	ramp = b4_estimate_ramp_height(100u, 901u);
	TEST_CHECK("B4-R03",
			   !ramp.valid && ramp.height_mm == B4_RAMP_INVALID_HEIGHT,
			   "angle above 90 degrees returns invalid result");

	/* B4-R04: zero distance is rejected. */
	ramp = b4_estimate_ramp_height(0u, 300u);
	TEST_CHECK("B4-R04",
			   !ramp.valid && ramp.height_mm == B4_RAMP_INVALID_HEIGHT,
			   "zero ramp distance returns invalid result");

	/* B4-R05: input beyond the configured distance bound is rejected. */
	ramp = b4_estimate_ramp_height(B4_RAMP_MAX_DISTANCE_MM + 1u, 300u);
	TEST_CHECK("B4-R05",
			   !ramp.valid && ramp.height_mm == B4_RAMP_INVALID_HEIGHT,
			   "distance beyond supported bound returns invalid result");

	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
