/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem4/TEST_LOG.md and the
 *	INT- result in testing/integration/TEST_LOG.md.
 */

#include "buddy4_data.h"
#include "buddy4_classifier.h"
#include "buddy4_ramp.h"
#include "buddy4_handling.h"
#include "buddy4_hump.h"
#include "subsystem4.h"
#include "../test_report.h"

#if APP_TEST

/* Feed the same synthetic sample n times to the handling detector. */
LOCAL void feed(B4_HANDLING *h, INT n, int16_t x, int16_t y, int16_t z)
{
	INT i;

	for (i = 0; i < n; i++) {
		b4_handling_update(h, x, y, z);
	}
}

/* Feed n samples; return 1 if LIFTED became set during them. */
LOCAL INT feed_lift_seen(B4_HANDLING *h, INT n, int16_t x, int16_t y,
			 int16_t z)
{
	INT i;
	INT seen = 0;

	for (i = 0; i < n; i++) {
		b4_handling_update(h, x, y, z);
		if (h->b_lifted != 0u) {
			seen = 1;
		}
	}
	return seen;
}

/*
 * Lift / overturn detector tests (change of requirement, week 12).
 * Synthetic 10 ms samples in calibrated mg; X right, Y rear, Z up.
 */
LOCAL void handling_tests(INT *pass, INT *fail)
{
	B4_HANDLING h;
	INT i;
	INT seen;
	INT test_pass_ = 0, test_fail_ = 0;

	/* B4-H01: level and still for 3 s: neither lifted nor overturned */
	b4_handling_init(&h);
	feed(&h, 300, 0, 0, 1000);
	TEST_CHECK("B4-H01", !h.b_lifted && !h.b_overturned,
		   "robot at rest is not lifted or overturned");

	/* B4-H02: 3 s of strong motor/floor vibration (+/-300 mg on Z,
	   +/-200 mg on X) must not look like a lift */
	b4_handling_init(&h);
	seen = 0;
	for (i = 0; i < 150; i++) {
		seen |= feed_lift_seen(&h, 1, 200, 0, 1300);
		seen |= feed_lift_seen(&h, 1, -200, 0, 700);
	}
	TEST_CHECK("B4-H02", !seen, "vibration does not trigger LIFTED");

	/* B4-H03: a hump landing jolt (+0.5 g for 30 ms, then the rebound)
	   ends with the chassis at rest, so it must not look like a lift */
	b4_handling_init(&h);
	feed(&h, 50, 0, 0, 1000);
	seen = feed_lift_seen(&h, 3, 0, 0, 1500);
	seen |= feed_lift_seen(&h, 3, 0, 0, 500);
	seen |= feed_lift_seen(&h, 100, 0, 0, 1000);
	TEST_CHECK("B4-H03", !seen, "short bump does not trigger LIFTED");

	/* B4-H04: lifting (+0.3 g for 200 ms) sets LIFTED within 200 ms */
	b4_handling_init(&h);
	feed(&h, 50, 0, 0, 1000);
	seen = feed_lift_seen(&h, 20, 0, 0, 1300);
	TEST_CHECK("B4-H04", seen, "lift is detected within 200 ms");

	/* B4-H05: a gentler lift (+0.1 g for 500 ms) also sets LIFTED */
	b4_handling_init(&h);
	feed(&h, 50, 0, 0, 1000);
	seen = feed_lift_seen(&h, 50, 0, 0, 1100);
	TEST_CHECK("B4-H05", seen,
		   "gentle lift is detected without a fast upward motion");

	/* B4-H06: LIFTED is held while carried, then clears after ~2 s */
	feed(&h, 100, 0, 0, 1000);
	i = h.b_lifted;
	feed(&h, 200, 0, 0, 1000);
	TEST_CHECK("B4-H06", i && !h.b_lifted,
		   "LIFTED latches for the hold time, then clears");

	/* B4-H07: upside down for 200 ms sets OVERTURNED */
	b4_handling_init(&h);
	feed(&h, 50, 0, 0, 1000);
	feed(&h, 25, 0, 0, -1000);
	TEST_CHECK("B4-H07", h.b_overturned, "upside down is OVERTURNED");

	/* B4-H08: on its side (90 deg roll) is also OVERTURNED */
	b4_handling_init(&h);
	feed(&h, 25, 1000, 0, 0);
	TEST_CHECK("B4-H08", h.b_overturned, "on its side is OVERTURNED");

	/* B4-H09: back upright for 0.5 s clears OVERTURNED */
	feed(&h, 60, 0, 0, 1000);
	TEST_CHECK("B4-H09", !h.b_overturned, "upright again clears OVERTURNED");

	/* B4-H10: a 20 deg hump slope held for 3 s is not OVERTURNED */
	b4_handling_init(&h);
	feed(&h, 300, 0, -342, 940);
	TEST_CHECK("B4-H10", !h.b_overturned && !h.b_lifted,
		   "20 deg slope is not overturned or lifted");

	/* B4-H11: integer square root used for |a| */
	TEST_CHECK("B4-H11",
		   b4_isqrt_u32(1000000u) == 1000u &&
		   b4_isqrt_u32(999999u) == 999u &&
		   b4_accel_magnitude_mg(-32768, -32768, -32768) == 56755,
		   "integer square root and magnitude are exact");

	*pass += test_pass_;
	*fail += test_fail_;
}

/*
 * Synthetic right-wheel hump: 10 mm high, 95 mm up, 5 mm flat, 95 mm
 * down, driven at 1 mm per 10 ms sample (10 cm/s). Returns the hump
 * height at wheel position pos_mm, in 0.1 mm, scaled by scale_pct.
 */
LOCAL int32_t hump_profile_dmm(int32_t pos_mm, int32_t scale_pct)
{
	int32_t h;

	if (pos_mm < 0 || pos_mm >= 195) {
		h = 0;
	} else if (pos_mm < 95) {
		h = (pos_mm * 100) / 95;
	} else if (pos_mm < 100) {
		h = 100;
	} else {
		h = 100 - ((pos_mm - 100) * 100) / 95;
	}
	return (h * scale_pct) / 100;
}

/*
 * Drive over the synthetic hump (car geometry from buddy4_hump.h).
 * vib_mg alternates sign every sample; move = 0 keeps the wheel still.
 * Returns the number of humps reported.
 */
LOCAL INT hump_drive(B4_HUMP *h, int32_t scale_pct, int16_t vib_mg,
		     int32_t move)
{
	INT i;
	INT found = 0;
	int32_t pos = (move == 0) ? 97 : -100;	/* still: sit on the top */
	int32_t hd;
	int16_t v;

	b4_hump_init(h);
	for (i = 0; i < 100; i++) {		/* 1 s level at start-up */
		(void)b4_hump_update(h, 20, -10, 1000, 0, 0, 10);
	}
	for (i = 0; i < 400; i++) {
		hd = hump_profile_dmm(pos, scale_pct);
		v = (int16_t)((i & 1) ? vib_mg : -vib_mg);
		found += b4_hump_update(h,
			(int16_t)(20 + (hd * 1000) / (10 * B4_HUMP_TRACK_MM) + v),
			(int16_t)(-10 + (hd * 1000) / (20 * B4_HUMP_CASTER_MM)),
			(int16_t)(1000 + v),
			0, pos + 100, 10);
		pos += move;
	}
	return found;
}

/* Hump height tests (right-wheel hump, roll based), synthetic data. */
LOCAL void hump_tests(INT *pass, INT *fail)
{
	/* static: B4_HUMP is ~250 bytes and the tests run in the initial
	   task, whose stack is only 1 KB (INITTASK_STKSZ). */
	static B4_HUMP h;
	INT i;
	INT found;
	INT test_pass_ = 0, test_fail_ = 0;

	/* B4-K01: 10 mm hump at 10 cm/s is measured as 10 +/- 1 mm */
	found = hump_drive(&h, 100, 0, 1);
	TEST_CHECK("B4-K01",
		   found == 1 && h.result_dmm >= 90 && h.result_dmm <= 110,
		   "10 mm right-wheel hump measured within 1 mm");

	/* B4-K02: same hump with +/-150 mg vibration */
	found = hump_drive(&h, 100, 150, 1);
	TEST_CHECK("B4-K02",
		   found == 1 && h.result_dmm >= 90 && h.result_dmm <= 110,
		   "hump measured within 1 mm with motor vibration");

	/* B4-K03: tilt without the wheel moving (held / bench) is not a hump */
	found = hump_drive(&h, 100, 0, 0);
	TEST_CHECK("B4-K03", found == 0,
		   "tilt with no right-wheel travel is not reported");

	/* B4-K04: flat floor with vibration reports no hump */
	found = hump_drive(&h, 0, 200, 1);
	TEST_CHECK("B4-K04", found == 0, "flat floor gives no hump");

	/* B4-K05: a 3 mm bump is below the minimum and is ignored */
	found = hump_drive(&h, 30, 0, 1);
	TEST_CHECK("B4-K05", found == 0, "3 mm bump is ignored");

	/* B4-K06: a turn adds 60 mg on X; with lat_comp_mg cancelling it the
	   reading stays level and no hump is reported */
	b4_hump_init(&h);
	for (i = 0; i < 100; i++) {
		(void)b4_hump_update(&h, 20, -10, 1000, 0, 0, 10);
	}
	found = 0;
	for (i = 0; i < 200; i++) {
		found += b4_hump_update(&h, 20 + 60, -10, 1000, -60, i, 10);
	}
	TEST_CHECK("B4-K06", found == 0 && h.now_dmm > -10 && h.now_dmm < 10,
		   "turn acceleration removed by lat_comp_mg");

	/* B4-K08: car stops halfway up the ramp for 8 s (line search), then
	   carries on: still one hump, within 1 mm */
	b4_hump_init(&h);
	for (i = 0; i < 100; i++) {
		(void)b4_hump_update(&h, 20, -10, 1000, 0, 0, 10);
	}
	{
		int32_t pos = -100;
		int32_t hd;
		INT t;

		found = 0;
		for (t = 0; t < 1200; t++) {
			hd = hump_profile_dmm(pos, 100);
			found += b4_hump_update(&h,
				(int16_t)(20 + (hd * 1000) / (10 * B4_HUMP_TRACK_MM)),
				(int16_t)(-10 - (hd * 1000) / (20 * B4_HUMP_CASTER_MM)),
				1000, 0, pos + 100, 10);
			if (pos != 50 || t > 950) {
				pos++;
			}
		}
	}
	TEST_CHECK("B4-K08",
		   found == 1 && h.result_dmm >= 90 && h.result_dmm <= 110,
		   "stopping on the ramp does not lose or skew the hump");

	/* B4-K09: after a reset (lift / read error) and a re-learn, level
	   reads level again: the averaging window used to keep stale values
	   after a reset, which turned later readings into garbage */
	b4_hump_init(&h);
	for (i = 0; i < 100; i++) {
		(void)b4_hump_update(&h, 20, -10, 1000, 0, i, 30);
	}
	for (i = 0; i < 60; i++) {
		b4_hump_relevel(&h);		/* held up: lifted */
		(void)b4_hump_update(&h, -450, -10, 880, 0, 100, 30);
	}
	for (i = 0; i < 120; i++) {
		(void)b4_hump_update(&h, 20, -10, 1000, 0, 100 + i, 30);
	}
	TEST_CHECK("B4-K09", h.ready && h.now_dmm > -10 && h.now_dmm < 10,
		   "level reads level after a lift and re-learn");

	/* B4-K07: height helper: 90 mg at 1 g on a 111 mm track ~ 10 mm */
	TEST_CHECK("B4-K07",
		   b4_hump_height_dmm(B4_HUMP_TRACK_MM, 90, 1000) == 100,
		   "roll ratio to height conversion");

	*pass += test_pass_;
	*fail += test_fail_;
}

EXPORT void subsystem4_test(void)
{
	T_RMBF rmbf;
	B4_MOTION_INPUT input = {0};
	B4_EVENT classified;
	B4_RAMP_RESULT ramp;

	TEST_INIT();

	/* B4-U01: state and flags fit into one byte. */
	B4_EVENT event = b4_pack_event(
		B4_STATE_TURNING, 1u, 0u);

	TEST_CHECK("B4-U01",
			   event == 0x0Bu,
			   "turning state plus overturned flag packs correctly");

	/* B4-U02: state can be decoded. */
	TEST_CHECK("B4-U02",
			   b4_event_state(event) == B4_STATE_TURNING,
			   "motion state can be extracted");

	/* B4-U03: overturned and IMU fault flags decode correctly. */
	TEST_CHECK("B4-U03",
			   b4_event_is_overturned(event) &&
				   !b4_event_has_imu_fault(event),
			   "overturned and IMU fault flags are independent");

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

	/*
	 * B4-U08: A stationary sensor tilted sideways
	 * should not be classified as climbing or descending.
	 */
	input.motion_level_mg = 0;
	input.vertical_delta_mg = 900;
	input.accel_x_mg = 900;
	input.accel_y_mg = 0;
	input.accel_z_mg = 435;
	input.yaw_rate_mdps = 0;
	input.imu_fault = 0u;

	classified = b4_classify_motion(input);

	TEST_CHECK("B4-U08",
			   b4_event_state(classified) != B4_STATE_CLIMBING &&
				   b4_event_state(classified) != B4_STATE_DESCENDING,
			   "sideways sensor tilt does not indicate climbing or descending");

	/*
	 * B4-U09: A stationary sensor tilted in the opposite
	 * direction should also avoid climbing/descending classification.
	 */
	input.motion_level_mg = 0;
	input.vertical_delta_mg = -900;
	input.accel_x_mg = -900;
	input.accel_y_mg = 0;
	input.accel_z_mg = 435;
	input.yaw_rate_mdps = 0;
	input.imu_fault = 0u;

	classified = b4_classify_motion(input);

	TEST_CHECK("B4-U09",
			   b4_event_state(classified) != B4_STATE_CLIMBING &&
				   b4_event_state(classified) != B4_STATE_DESCENDING,
			   "opposite sensor tilt does not indicate climbing or descending");

	/*
	 * B4-U10 (changed, week 12): collision detection removed.
	 * A lifted robot is classified as LIFTED (was: impact).
	 */
	input.motion_level_mg = 100;
	input.lifted = 1u;
	input.overturned = 0u;
	input.vertical_delta_mg = 0;
	input.accel_x_mg = 0;
	input.accel_y_mg = 0;
	input.accel_z_mg = 1000;
	input.yaw_rate_mdps = 0;
	input.imu_fault = 0u;

	classified = b4_classify_motion(input);

	TEST_CHECK("B4-U10",
			   b4_event_state(classified) == B4_STATE_LIFTED &&
				   !b4_event_is_overturned(classified),
			   "lifted robot is classified as LIFTED");

	/* B4-U11: An IMU fault should produce a fault event. */
	input.lifted = 0u;
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
	input.motion_level_mg = B4_MOTION_THRESHOLD_MG - 1;
	input.vertical_delta_mg = 0;
	input.accel_x_mg = 0;
	input.accel_y_mg = 0;
	input.accel_z_mg = 0;
	input.yaw_rate_mdps = 0;
	input.imu_fault = 0u;

	classified = b4_classify_motion(input);
	TEST_CHECK("B4-U12",
			   b4_event_state(classified) == B4_STATE_STATIONARY,
			   "activity just below threshold is stationary");

	/* B4-U13: Motion at the threshold should be moving. */
	input.motion_level_mg = B4_MOTION_THRESHOLD_MG;
	input.vertical_delta_mg = 0;
	input.accel_x_mg = 0;
	input.accel_y_mg = 0;
	input.accel_z_mg = 0;
	input.yaw_rate_mdps = 0;
	input.imu_fault = 0u;

	classified = b4_classify_motion(input);

	TEST_CHECK("B4-U13",
			   b4_event_state(classified) == B4_STATE_MOVING,
			   "activity at threshold is classified as moving");

	/* B4-U14: Negative yaw rate should also detect turning. */
	input.motion_level_mg = 100;
	input.vertical_delta_mg = 0;
	input.accel_x_mg = 0;
	input.accel_y_mg = 0;
	input.accel_z_mg = 0;
	input.yaw_rate_mdps = -B4_TURN_THRESHOLD_MDPS;
	input.imu_fault = 0u;

	classified = b4_classify_motion(input);

	TEST_CHECK("B4-U14",
			   b4_event_state(classified) == B4_STATE_TURNING,
			   "large negative yaw rate is classified as turning");

	/* B4-U15: An IMU fault takes priority over a lift condition. */
	input.motion_level_mg = 100;
	input.lifted = 1u;
	input.vertical_delta_mg = 0;
	input.accel_x_mg = 0;
	input.accel_y_mg = 0;
	input.accel_z_mg = 1000;
	input.yaw_rate_mdps = 0;
	input.imu_fault = 1u;

	classified = b4_classify_motion(input);

	TEST_CHECK("B4-U15",
			   b4_event_state(classified) == B4_STATE_FAULT &&
				   b4_event_has_imu_fault(classified) &&
				   b4_event_state(classified) != B4_STATE_LIFTED,
			   "IMU fault takes priority over lift classification");

	/* B4-U17: overturned flag is reported with the motion state. */
	input.imu_fault = 0u;
	input.lifted = 0u;
	input.overturned = 1u;
	input.motion_level_mg = 0;
	input.yaw_rate_mdps = 0;
	classified = b4_classify_motion(input);
	TEST_CHECK("B4-U17",
			   b4_event_is_overturned(classified) &&
				   b4_event_state(classified) == B4_STATE_STATIONARY,
			   "overturned flag is carried with the motion state");

	/* B4-U16: Reserved bits must remain zero. */
	classified = b4_pack_event(
		B4_STATE_TURNING,
		1u,
		1u);

	TEST_CHECK("B4-U16",
			   (classified & B4_RESERVED_MASK) == 0u,
			   "reserved bits remain zero");

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

	handling_tests(&test_pass_, &test_fail_);
	hump_tests(&test_pass_, &test_fail_);

	/* S4-U02: IF-13 flag exists and holds only the documented bits */
	{
		T_RFLG rflg;
		TEST_CHECK("S4-U02",
			   tk_ref_flg(flg_s4_imu, &rflg) == E_OK &&
			   (rflg.flgptn & ~S4_FLG_ALL) == 0u,
			   "flg_s4_imu exists and uses only S4_FLG_* bits");
	}

	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */