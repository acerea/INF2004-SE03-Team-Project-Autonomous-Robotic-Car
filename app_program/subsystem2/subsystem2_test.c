/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.
 */

#include "pid_control.h"
#include "wheel_encoder.h"
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

		/* ====================================================================================
		* S2-M01: Basic Motor Movement Test
		*
		* Tests:
		* - Move forward
		* - Move backward
		* - Turn left
		* - Turn right
		*
		* Change '#if 0' to '#if 1' to enable the test
		* ===================================================================================== */
		#if 0
		{
			tm_printf((UB*)"[S2-M01] Starting motor movement test\n");

			tk_dly_tsk(5000);

			tm_printf((UB*)"[S2-M01] Moving forward\n");
			motion_moveForward(40);
			tk_dly_tsk(300);
			motion_stop();

			tm_printf((UB*)"[S2-M01] Moving backward\n");
			motion_moveBackward(40);
			tk_dly_tsk(300);
			motion_stop();

			tm_printf((UB*)"[S2-M01] Turning left\n");
			motion_turnLeft(40);
			tk_dly_tsk(300);
			motion_stop();

			tm_printf((UB*)"[S2-M01] Turning right\n");
			motion_turnRight(40);
			tk_dly_tsk(300);
			motion_stop();

			tm_printf((UB*)"[S2-M01] Motor test completed\n");
		}
		#endif

		/* ====================================================================================
		* S2-E01: Manual Encoder Counting Test
		*
		* Tests:
		* - Rotate wheels manually and observe encoder counts
		*
		* Change '#if 0' to '#if 1' to enable the test
		* ===================================================================================== */
		#if 0
    	{
			int i;

			tm_printf((UB*)"[S2-E01] Manual encoder count test\n");

			tk_dly_tsk(5000);

			for (i = 0; i < 100; ++i) {

				tm_printf((UB*)"[S2-E01] Left: %ld | Right: %ld\n",
						(long)encoder_getLeftCount(),
						(long)encoder_getRightCount());

				tk_dly_tsk(200);
			}

			tm_printf((UB*)"[S2-E01] Test completed\n");
		}
		#endif

		/* ====================================================================================
		* S2-E02: Preliminary CPR Calibration Test
		*
		* Tests:
		* - Rotate five full revolutions of each wheel and observe encoder counts
		*
		* Change '#if 0' to '#if 1' to enable the test
		* ===================================================================================== */
		#if 0
		{
			int32_t l0, r0, dl, dr;

			tm_printf((UB*)"[S2-E02] Starting CPR calibration test\n");

			tk_dly_tsk(5000);

			/* LEFT WHEEL */
			tm_printf((UB*)"[S2-E02] Rotate LEFT wheel 5 full turns\n");

			l0 = encoder_getLeftCount();

			tk_dly_tsk(30000);

			dl = encoder_getLeftCount() - l0;

			tm_printf((UB*)"[S2-E02] Left five turns: %ld\n",
					(long)dl);

			tm_printf((UB*)"[S2-E02] Left CPR: %ld\n",
					(long)(dl / 5));

			/* RIGHT WHEEL */
			tm_printf((UB*)"[S2-E02] Rotate RIGHT wheel 5 full turns\n");

			r0 = encoder_getRightCount();

			tk_dly_tsk(30000);

			dr = encoder_getRightCount() - r0;

			tm_printf((UB*)"[S2-E02] Right five turns: %ld\n",
					(long)dr);

			tm_printf((UB*)"[S2-E02] Right CPR: %ld\n",
					(long)(dr / 5));

			tm_printf((UB*)"[S2-E02] Calibration completed\n");
		}
		#endif

		/* ====================================================================================
		* S2-E03: Motor-driven Encoder Test
		*
		* Tests:
		* - Encoder counting during motor operation
		*
		* Change '#if 0' to '#if 1' to enable the test
		* ===================================================================================== */
		#if 0
		{
			tm_printf((UB*)"[S2-E03] Starting motor-driven encoder test\n");

			tk_dly_tsk(5000);

			int32_t left_before = encoder_getLeftCount();
			int32_t right_before = encoder_getRightCount();

			/* Run motors briefly */
			motion_moveForward(40);
			tk_dly_tsk(500);
			motion_stop();

			/* Read counts after movement */
			int32_t left_after = encoder_getLeftCount();
			int32_t right_after = encoder_getRightCount();

			tm_printf((UB*)"[S2-E03] Left count change: %ld\n",
					(long)(left_after - left_before));

			tm_printf((UB*)"[S2-E03] Right count change: %ld\n",
					(long)(right_after - right_before));

			tm_printf((UB*)"[S2-E03] Test completed\n");
		}
		#endif 

		/* ====================================================================================
		* S2-S01: Speed Estimation using Measured Time Test
		*
		* Change '#if 0' to '#if 1' to enable the test
		* ===================================================================================== */
		#if 0
		{
		tm_printf((UB*)"[S2-S01] Starting motor-driven speed test\n");

		/* Allow time to connect the serial monitor */
		tk_dly_tsk(5000);

		SYSTIM start_time;
		SYSTIM end_time;

		/* Start motors */
		motion_moveForward(40);

		/* Record start time and encoder counts */
		tk_get_tim(&start_time);
		int32_t left_before = encoder_getLeftCount();
		int32_t right_before = encoder_getRightCount();

		/* Allow wheels to rotate */
		tk_dly_tsk(200);

		/* Record final counts and time */
		int32_t left_after = encoder_getLeftCount();
		int32_t right_after = encoder_getRightCount();
		tk_get_tim(&end_time);

		tk_get_tim(&end_time);

		/* Stop both motors */
		motion_stop();

		/* Calculate encoder count changes */
		int32_t left_change = left_after - left_before;
		int32_t right_change = right_after - right_before;

		/* Combine high and low parts of system time */
		uint64_t start_ms =
			((uint64_t)(uint32_t)start_time.hi << 32) | start_time.lo;

		uint64_t end_ms =
			((uint64_t)(uint32_t)end_time.hi << 32) | end_time.lo;

		/* Calculate actual elapsed seconds */
		float dt_seconds = (float)(end_ms - start_ms) / 1000.0f;

		/* Calculate wheel speeds */
		float left_speed =
			encoder_calculateLeftSpeed(left_change, dt_seconds);

		float right_speed =
			encoder_calculateRightSpeed(right_change, dt_seconds);

		/* Convert for serial printing */
		int left_speed10 = (int)(left_speed * 10.0f);
		int right_speed10 = (int)(right_speed * 10.0f);

		tm_printf((UB*)"[S2-S01] Elapsed time: %lu ms\n",
				(unsigned long)(end_ms - start_ms));

		tm_printf((UB*)"[S2-S01] Left speed: %d.%01d cm/s\n",
				left_speed10 / 10, left_speed10 % 10);

		tm_printf((UB*)"[S2-S01] Right speed: %d.%01d cm/s\n",
				right_speed10 / 10, right_speed10 % 10);

		tm_printf((UB*)"[S2-S01] Test completed\n");
		}
		#endif 

		/* ====================================================================================
		* S2-S02: Motor-driven Speed Response Test
		*
		* Tests:
		* - Wheel speed at 40% PWM
		* - Then wheel speed at 60% PWM
		*
		* Change '#if 0' to '#if 1' to enable the test
		* ===================================================================================== */
		#if 0
		{
			const int pwms[2] = {40, 60};
			int i;

			tm_printf((UB*)"[S2-S02] Speed response: 40%% vs 60%% PWM\n");

			tk_dly_tsk(5000);

			for (i = 0; i < 2; ++i) {

				int32_t l0, r0, l1, r1;
				SYSTIM start, end;
				uint64_t start_ms, end_ms;
				float dt, ls, rs;

				motion_moveForward(pwms[i]);

				l0 = encoder_getLeftCount();
				r0 = encoder_getRightCount();
				tk_get_tim(&start);

				tk_dly_tsk(200);

				l1 = encoder_getLeftCount();
				r1 = encoder_getRightCount();
				tk_get_tim(&end);

				motion_stop();

				/* Convert SYSTIM to milliseconds */
				start_ms = ((uint64_t)(uint32_t)start.hi << 32)
						| (uint32_t)start.lo;

				end_ms = ((uint64_t)(uint32_t)end.hi << 32)
					| (uint32_t)end.lo;

				dt = (float)(end_ms - start_ms) / 1000.0f;

				ls = encoder_calculateLeftSpeed(l1 - l0, dt);
				rs = encoder_calculateRightSpeed(r1 - r0, dt);

				tm_printf((UB*)"[S2-S02] PWM=%d%%, elapsed=%lu ms\n",
						pwms[i], (unsigned long)(end_ms - start_ms));

				tm_printf((UB*)"[S2-S02] Left speed: %d.%01d cm/s\n",
						(int)ls, ((int)(ls * 10)) % 10);

				tm_printf((UB*)"[S2-S02] Right speed: %d.%01d cm/s\n",
						(int)rs, ((int)(rs * 10)) % 10);

				tk_dly_tsk(1000);
			}

			tm_printf((UB*)"[S2-S02] Test completed\n");
		}
		#endif

		/* ====================================================================================
		* S2-S03: Manual Speed Estimation Test
		*
		* Tests:
		* - Estimate wheel speed by manually rotating the wheels and observing encoder counts
		*
		* Change '#if 0' to '#if 1' to enable the test
		* ===================================================================================== */
		#if 0
		{
			int32_t l0, r0, l1, r1;
			float ls, rs;

			tm_printf((UB*)"[S2-S01] Manual speed estimation test\n");
			tk_dly_tsk(5000);

			l0 = encoder_getLeftCount();
			r0 = encoder_getRightCount();

			tm_printf((UB*)"[S2-S01] Rotate wheels for 5 seconds\n");
			tk_dly_tsk(5000);

			l1 = encoder_getLeftCount();
			r1 = encoder_getRightCount();

			ls = encoder_calculateLeftSpeed(l1 - l0, 5.0f);
			rs = encoder_calculateRightSpeed(r1 - r0, 5.0f);

			tm_printf((UB*)"[S2-S01] Left speed: %d cm/s\n", (int)ls);
			tm_printf((UB*)"[S2-S01] Right speed: %d cm/s\n", (int)rs);

			/* Stationary check */
			l0 = encoder_getLeftCount();
			r0 = encoder_getRightCount();

			tm_printf((UB*)"[S2-S01] Stop rotating both wheels\n");

			/* Give time to stop the wheels */
			tk_dly_tsk(5000);

			/* Begin stationary measurement */
			l0 = encoder_getLeftCount();
			r0 = encoder_getRightCount();

			tk_dly_tsk(5000);

			l1 = encoder_getLeftCount();
			r1 = encoder_getRightCount();

			ls = encoder_calculateLeftSpeed(l1 - l0, 5.0f);
			rs = encoder_calculateRightSpeed(r1 - r0, 5.0f);

			tm_printf((UB*)"[S2-S01] Stationary left speed: %d cm/s\n", (int)ls);
			tm_printf((UB*)"[S2-S01] Stationary right speed: %d cm/s\n", (int)rs);

			tm_printf((UB*)"[S2-S01] Test completed\n");
		}
		#endif

		/* ====================================================================================
		* S2-ED01: Motor-driven Encoder Distance Test
		*
		* Tests:
		* - Encoder counting during motor operation
 		* - Encoder-based wheel distance estimation
		*
		* Change '#if 0' to '#if 1' to enable the test
		* ===================================================================================== */
		#if 0
		{
		tm_printf((UB*)"[S2-ED01] Starting motor-driven distance test\n");

		tk_dly_tsk(5000);

		/* Read counts before movement */
		int32_t left_before = encoder_getLeftCount();
		int32_t right_before = encoder_getRightCount();

		float left_distance_before = encoder_getLeftDistance();
		float right_distance_before = encoder_getRightDistance();

		/* Move forward at 40% PWM for 500 ms */
		motion_moveForward(40);
		tk_dly_tsk(500);
		motion_stop();

		tk_dly_tsk(500);

		/* Read counts after movement */
		int32_t left_after = encoder_getLeftCount();
		int32_t right_after = encoder_getRightCount();

		/* Calculate encoder count changes */
		int32_t left_change = left_after - left_before;
		int32_t right_change = right_after - right_before;

		tm_printf((UB*)"[S2-ED01] Left count change: %ld\n",
				(long)left_change);

		tm_printf((UB*)"[S2-ED01] Right count change: %ld\n",
				(long)right_change);

		/* Calculate distance travelled during the test (cm) */
		float left_distance = encoder_getLeftDistance() - left_distance_before;
		float right_distance = encoder_getRightDistance() - right_distance_before;

		/* Convert to cm */
		int left_distance10 = (int)(left_distance * 10.0f);
		int right_distance10 = (int)(right_distance * 10.0f);

		tm_printf((UB*)"[S2-ED01] Left distance: %d.%01d cm\n",
				left_distance10 / 10,
				left_distance10 % 10);

		tm_printf((UB*)"[S2-ED01] Right distance: %d.%01d cm\n",
				right_distance10 / 10,
				right_distance10 % 10);

		tm_printf((UB*)"[S2-ED01] Test completed\n");
		}
		#endif 

		/* ====================================================================================
		* S2-M02: Independent Wheel PWM Control Test
		*
		* Tests:
		* - Set left wheel to 40% PWM and right wheel to 60% PWM
		*
		* Change '#if 0' to '#if 1' to enable the test
		* ===================================================================================== */
		#if 1
		{
			tm_printf((UB*)"[S2-M02] Independent wheel PWM test\n");

			tk_dly_tsk(5000);

			tm_printf((UB*)"[S2-M02] Left=40%%, Right=60%%\n");
			motion_setWheelPWM(40, 60);

			tk_dly_tsk(2000);
			motion_stop();

			tm_printf((UB*)"[S2-M02] Test completed\n");
		}
		#endif

		/* ====================================================================================
		* S2-P01: P Controller PWM Calculation Test
		*
		* Tests:
		* - Verify that the speed_p_update function calculates the correct PWM output based on 
		* target speed, measured speed, base PWM, and proportional gain (kp).
		*
		* Change '#if 0' to '#if 1' to enable the test
		* ===================================================================================== */
		#if 0
		{
			float pwm;

			tm_printf((UB*)"[S2-P01] P controller calculation test\n");

			pwm = speed_p_update(15.0f, 12.0f, 40.0f, 2.0f);

			tm_printf((UB*)"[S2-P01] Calculated PWM: %d%%\n",
					(int)pwm);

			if ((int)pwm == 46) {
				tm_printf((UB*)"[S2-P01] PASS\n");
			} else {
				tm_printf((UB*)"[S2-P01] FAIL\n");
			}
		}
		#endif

		/* ====================================================================================
		* S2-P02: P Control with Encoder Feedback Test
		*
		* Tests:
		* - Verify that the proportional control algorithm adjusts the PWM output based on 
		*   encoder feedback to maintain the target speed.
		*
		* Change '#if 0' to '#if 1' to enable the test
		* ===================================================================================== */
		#if 0
		{
			const float target_speed = 15.0f;
			const float base_pwm = 40.0f;
			const float kp = 2.0f;

			int i;
			int32_t l0, r0, l1, r1;
			SYSTIM start, end;
			uint64_t start_ms, end_ms;
			float dt, left_speed, right_speed;
			float left_pwm = base_pwm;
			float right_pwm = base_pwm;

			tm_printf((UB*)"[S2-P02] P control feedback test\n");
			tk_dly_tsk(5000);

			motion_setWheelPWM((int)left_pwm, (int)right_pwm);

			for (i = 0; i < 10; i++) {
				l0 = encoder_getLeftCount();
				r0 = encoder_getRightCount();
				tk_get_tim(&start);

				tk_dly_tsk(200);

				l1 = encoder_getLeftCount();
				r1 = encoder_getRightCount();
				tk_get_tim(&end);

				start_ms = ((uint64_t)(uint32_t)start.hi << 32)
						| (uint32_t)start.lo;
				end_ms = ((uint64_t)(uint32_t)end.hi << 32)
					| (uint32_t)end.lo;

				dt = (float)(end_ms - start_ms) / 1000.0f;

				if (dt <= 0.0f) {
					tm_printf((UB*)"[S2-P02] Invalid elapsed time\n");
					break;
				}

				left_speed = encoder_calculateLeftSpeed(l1 - l0, dt);
				right_speed = encoder_calculateRightSpeed(r1 - r0, dt);

				left_pwm = speed_p_update(
					target_speed, left_speed, base_pwm, kp);

				right_pwm = speed_p_update(
					target_speed, right_speed, base_pwm, kp);

				motion_setWheelPWM((int)left_pwm, (int)right_pwm);

				tm_printf((UB*)"[S2-P02] Cycle %d: L PWM=%d%%, R PWM=%d%%\n",
						i + 1, (int)left_pwm, (int)right_pwm);
				tm_printf((UB*)"[S2-P02] L speed=%d, R speed=%d cm/s\n",
						(int)left_speed, (int)right_speed);
			}

			motion_stop();
			tm_printf((UB*)"[S2-P02] Test completed\n");
		}
		#endif

		/* ====================================================================================
		* S2-P03: PI Controller Calculation Test
		*
		* Tests:
		* - Verify that the speed_pi_update function calculates the correct PWM output based on 
		*   target speed, measured speed, base PWM, proportional gain (kp), integral gain (ki), 
		*   and accumulated integral error.
		*
		* Change '#if 0' to '#if 1' to enable the test
		* ===================================================================================== */
		#if 0
		{
			SpeedPI left_pi;
			float pwm1, pwm2;

			tm_printf((UB*)"[S2-P03] PI controller calculation test\n");

			speed_pi_init(&left_pi, 2.0f, 1.0f, 40.0f);

			pwm1 = speed_pi_update(&left_pi, 15.0f, 12.0f, 0.2f);
			pwm2 = speed_pi_update(&left_pi, 15.0f, 12.0f, 0.2f);

			tm_printf((UB*)"[S2-P03] First PWM: %d.%01d%%\n",
					(int)pwm1, ((int)(pwm1 * 10)) % 10);

			tm_printf((UB*)"[S2-P03] Second PWM: %d.%01d%%\n",
					(int)pwm2, ((int)(pwm2 * 10)) % 10);

			if (pwm1 > 46.5f && pwm1 < 46.7f &&
				pwm2 > 47.1f && pwm2 < 47.3f) {
				tm_printf((UB*)"[S2-P03] PASS\n");
			} else {
				tm_printf((UB*)"[S2-P03] FAIL\n");
			}
		}
		#endif

		/* ====================================================================================
		* S2-P04: PI Control with Encoder Feedback Test
		*
		* Tests:
		* - Verify that the PI control algorithm adjusts the PWM output based on encoder 
		*   feedback to maintain the target speed.
		*
		* Change '#if 0' to '#if 1' to enable the test
		* ===================================================================================== */
		#if 0
		{
			const float target_speed = 15.0f;
			const float base_pwm = 30.0f;
			const float kp = 2.0f;
			const float ki = 1.0f;

			SpeedPI left_pi, right_pi;

			int i;
			int32_t l0, r0, l1, r1;
			SYSTIM start, end;
			uint64_t start_ms, end_ms;
			float dt, left_speed, right_speed;
			float left_pwm = base_pwm;
			float right_pwm = base_pwm;

			tm_printf((UB*)"[S2-P04] PI encoder feedback test\n");

			speed_pi_init(&left_pi, kp, ki, base_pwm);
			speed_pi_init(&right_pi, kp, ki, base_pwm);

			tk_dly_tsk(5000);

			motion_setWheelPWM((int)left_pwm, (int)right_pwm);

			for (i = 0; i < 10; i++) {

				l0 = encoder_getLeftCount();
				r0 = encoder_getRightCount();
				tk_get_tim(&start);

				tk_dly_tsk(200);

				l1 = encoder_getLeftCount();
				r1 = encoder_getRightCount();
				tk_get_tim(&end);

				start_ms = ((uint64_t)(uint32_t)start.hi << 32)
						| (uint32_t)start.lo;

				end_ms = ((uint64_t)(uint32_t)end.hi << 32)
					| (uint32_t)end.lo;

				if (end_ms <= start_ms) {
					tm_printf((UB*)"[S2-P04] Invalid elapsed time\n");
					break;
				}

				dt = (float)(end_ms - start_ms) / 1000.0f;

				left_speed = encoder_calculateLeftSpeed(l1 - l0, dt);
				right_speed = encoder_calculateRightSpeed(r1 - r0, dt);

				left_pwm = speed_pi_update(
					&left_pi, target_speed, left_speed, dt);

				right_pwm = speed_pi_update(
					&right_pi, target_speed, right_speed, dt);

				motion_setWheelPWM((int)left_pwm, (int)right_pwm);

				tm_printf((UB*)"[S2-P04] Cycle %d: L PWM=%d%%, R PWM=%d%%\n",
						i + 1, (int)left_pwm, (int)right_pwm);

				tm_printf((UB*)"[S2-P04] L speed=%d, R speed=%d cm/s\n",
						(int)left_speed, (int)right_speed);
			}

			motion_stop();

			tm_printf((UB*)"[S2-P04] Test completed\n");
		}
		#endif

		/* ====================================================================================
		* S2-P05: Full PID Calculation Test
		*
		* Change '#if 0' to '#if 1' to enable the test
		* ===================================================================================== */
		#if 0
		{
			SpeedPID pid;
			float pwm1, pwm2, pwm3;

			tm_printf((UB*)"[S2-P05] PID calculation test\n");

			speed_pid_init(&pid, 2.0f, 1.0f, 0.1f, 40.0f);

			pwm1 = speed_pid_update(&pid, 15.0f, 12.0f, 0.2f);
			pwm2 = speed_pid_update(&pid, 15.0f, 13.0f, 0.2f);
			pwm3 = speed_pid_update(&pid, 15.0f, 14.0f, 0.2f);

			tm_printf((UB*)"[S2-P05] PWM1=%d.%01d%%\n",
					(int)pwm1, ((int)(pwm1 * 10)) % 10);

			tm_printf((UB*)"[S2-P05] PWM2=%d.%01d%%\n",
					(int)pwm2, ((int)(pwm2 * 10)) % 10);

			tm_printf((UB*)"[S2-P05] PWM3=%d.%01d%%\n",
					(int)pwm3, ((int)(pwm3 * 10)) % 10);

			if (pwm1 > 46.5f && pwm1 < 46.7f &&
				pwm2 > 44.4f && pwm2 < 44.6f &&
				pwm3 > 42.6f && pwm3 < 42.8f) {
				tm_printf((UB*)"[S2-P05] PASS\n");
			} else {
				tm_printf((UB*)"[S2-P05] FAIL\n");
			}
		}
		#endif

		/* ====================================================================================
		* S2-P06: Full PID with Encoder Feedback Test
		*
		* Change '#if 0' to '#if 1' to enable the test
		* ===================================================================================== */
		#if 0
		{
			const float target_speed = 15.0f;
			const float base_pwm = 30.0f;
			const float kp = 2.0f;
			const float ki = 1.0f;
			const float kd = 0.01f;

			SpeedPID left_pid, right_pid;
			float left_pwm = base_pwm;
			float right_pwm = base_pwm;

			tm_printf((UB*)"[S2-P06] Full PID encoder feedback test\n");

			speed_pid_init(&left_pid, kp, ki, kd, base_pwm);
			speed_pid_init(&right_pid, kp, ki, kd, base_pwm);

			tk_dly_tsk(5000);

			motion_setWheelPWM((int)left_pwm, (int)right_pwm);

			for (int i = 0; i < 10; i++) {
				int32_t l0, r0, l1, r1;
				SYSTIM start, end;
				uint64_t start_ms, end_ms;
				float dt, left_speed, right_speed;

				l0 = encoder_getLeftCount();
				r0 = encoder_getRightCount();
				tk_get_tim(&start);

				tk_dly_tsk(200);

				l1 = encoder_getLeftCount();
				r1 = encoder_getRightCount();
				tk_get_tim(&end);

				start_ms = ((uint64_t)(uint32_t)start.hi << 32)
						| (uint32_t)start.lo;
				end_ms = ((uint64_t)(uint32_t)end.hi << 32)
					| (uint32_t)end.lo;

				if (end_ms <= start_ms) {
					tm_printf((UB*)"[S2-P06] Invalid elapsed time\n");
					break;
				}

				dt = (float)(end_ms - start_ms) / 1000.0f;

				left_speed = encoder_calculateLeftSpeed(l1 - l0, dt);
				right_speed = encoder_calculateRightSpeed(r1 - r0, dt);

				left_pwm = speed_pid_update(
					&left_pid, target_speed, left_speed, dt);

				right_pwm = speed_pid_update(
					&right_pid, target_speed, right_speed, dt);

				motion_setWheelPWM((int)left_pwm, (int)right_pwm);

				tm_printf((UB*)"[S2-P06] Cycle %d: L PWM=%d%%, R PWM=%d%%\n",
						i + 1, (int)left_pwm, (int)right_pwm);

				tm_printf((UB*)"[S2-P06] L speed=%d, R speed=%d cm/s\n",
						(int)left_speed, (int)right_speed);
			}

			motion_stop();
			tm_printf((UB*)"[S2-P06] Test completed\n");
		}
		#endif

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
