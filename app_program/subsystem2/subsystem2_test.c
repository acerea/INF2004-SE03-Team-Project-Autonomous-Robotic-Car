/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.
 */

#include "motion_pid.h"
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
		#if 0
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

		/* ============================================================
		* S2-P07: Integrated PID Wheel Speed Control Test (FR5)
		*
		* Tests:
		* - PID integration using motion_pid.c
		* - Encoder feedback and wheel speed response
		* - Target speed: 15 cm/s
		*
		* Change '#if 0' to '#if 1' to enable
		* ============================================================ */
		#if 0
		{
			SYSTIM previous_time, current_time;
			uint64_t previous_ms, current_ms;
			int32_t left_before, right_before;
			int32_t left_after, right_after;
			float dt, left_speed, right_speed;

			tm_printf((UB*)"[S2-P07] Integrated PID test\n");
			tm_printf((UB*)"[S2-P07] Target speed: 15 cm/s\n");

			tk_dly_tsk(5000);

			/* Initialise PID and set target speed */
			motion_pidInit();
			motion_pidSetTargetSpeed(15.0f);

			/* Begin at the preliminary base PWM */
			motion_setWheelPWM(30, 30);

			left_before = encoder_getLeftCount();
			right_before = encoder_getRightCount();

			tk_get_tim(&previous_time);

			previous_ms =
				((uint64_t)(uint32_t)previous_time.hi << 32)
				| (uint32_t)previous_time.lo;

			for (int i = 0; i < 10; i++) {

				tk_dly_tsk(200);

				tk_get_tim(&current_time);

				current_ms =
					((uint64_t)(uint32_t)current_time.hi << 32)
					| (uint32_t)current_time.lo;

				if (current_ms <= previous_ms) {
					tm_printf((UB*)"[S2-P07] Invalid elapsed time\n");
					break;
				}

				dt = (float)(current_ms - previous_ms) / 1000.0f;

				left_after = encoder_getLeftCount();
				right_after = encoder_getRightCount();

				left_speed = encoder_calculateLeftSpeed(
					left_after - left_before, dt);

				right_speed = encoder_calculateRightSpeed(
					right_after - right_before, dt);

				/* Integrated PID performs its own encoder feedback */
				motion_pidUpdate(dt);

				tm_printf((UB*)"[S2-P07] Cycle %d, dt=%lu ms\n",
						i + 1,
						(unsigned long)(current_ms - previous_ms));

				tm_printf((UB*)"[S2-P07] Left=%d, Right=%d cm/s\n",
						(int)left_speed,
						(int)right_speed);

				previous_ms = current_ms;
				left_before = left_after;
				right_before = right_after;
			}

			motion_pidStop();

			tm_printf((UB*)"[S2-P07] Test completed\n");
		}
		#endif

		/* ============================================================
		* S2-P08: S2-P08: PID Control Loop Timing Test (NFR2) 
		*
		* Change '#if 0' to '#if 1' to enable
		* ============================================================ */
		#if 0
		{
			SYSTIM previous, current;
			uint64_t previous_ms, current_ms;
			uint64_t min_ms = 0, max_ms = 0, total_ms = 0;
			uint64_t elapsed_ms;
			int completed = 0;

			tm_printf((UB*)"[S2-P08] PID timing test\n");
			tk_dly_tsk(5000);

			motion_pidInit();
			motion_pidSetTargetSpeed(15.0f);
			motion_setWheelPWM(30, 30);

			tk_get_tim(&previous);
			previous_ms =
				((uint64_t)(uint32_t)previous.hi << 32) |
				(uint32_t)previous.lo;

			for (int i = 0; i < 20; i++) {
				tk_dly_tsk(10);

				tk_get_tim(&current);
				current_ms =
					((uint64_t)(uint32_t)current.hi << 32) |
					(uint32_t)current.lo;

				if (current_ms <= previous_ms) {
					tm_printf((UB*)"[S2-P08] Invalid elapsed time\n");
					break;
				}

				elapsed_ms = current_ms - previous_ms;

				motion_pidUpdate((float)elapsed_ms / 1000.0f);

				if (completed == 0 || elapsed_ms < min_ms)
					min_ms = elapsed_ms;

				if (elapsed_ms > max_ms)
					max_ms = elapsed_ms;

				total_ms += elapsed_ms;
				completed++;

				previous_ms = current_ms;
			}

			motion_pidStop();

			if (completed > 0) {
				tm_printf((UB*)"[S2-P08] Min=%lu ms, Max=%lu ms, Avg=%lu ms\n",
						(unsigned long)min_ms,
						(unsigned long)max_ms,
						(unsigned long)(total_ms / completed));
			}

			tm_printf((UB*)"[S2-P08] Completed=%d/20\n", completed);

			tm_printf((UB*)"[S2-P08] NFR2 timing: %s\n",
					(completed == 20 && max_ms <= 20)
					? "PASS" : "FAIL");
		}
		#endif

		/* ============================================================
		* S2-P09: Straight-Line Motion Correction Test (FR5)
		*
		* Change '#if 0' to '#if 1' to enable
		* ============================================================ */
		#if 0
		{
			SYSTIM start, end;
			uint64_t start_ms, end_ms;
			int32_t l0, r0, l1, r1;
			float dt, left_speed, right_speed;
			float speed_difference;

			tm_printf((UB*)"[S2-P09] Straight-line correction test\n");
			tk_dly_tsk(5000);

			motion_pidInit();
			motion_pidSetTargetSpeed(15.0f);

			/* Begin with preliminary base PWM */
			motion_setWheelPWM(30, 30);

			l0 = encoder_getLeftCount();
			r0 = encoder_getRightCount();
			tk_get_tim(&start);

			for (int i = 0; i < 20; i++) {
				/* Initial test interval; not NFR2 validation */
				tk_dly_tsk(200);

				tk_get_tim(&end);
				end_ms = ((uint64_t)(uint32_t)end.hi << 32)
						| (uint32_t)end.lo;
				start_ms = ((uint64_t)(uint32_t)start.hi << 32)
						| (uint32_t)start.lo;

				if (end_ms <= start_ms) {
					tm_printf((UB*)"[S2-P09] Invalid elapsed time\n");
					break;
				}

				dt = (float)(end_ms - start_ms) / 1000.0f;

				l1 = encoder_getLeftCount();
				r1 = encoder_getRightCount();

				left_speed = encoder_calculateLeftSpeed(l1 - l0, dt);
				right_speed = encoder_calculateRightSpeed(r1 - r0, dt);

				speed_difference = left_speed - right_speed;
				if (speed_difference < 0.0f)
					speed_difference = -speed_difference;

				motion_pidUpdate(dt);

				tm_printf((UB*)"[S2-P09] Cycle %d: L=%d, R=%d, Diff=%d cm/s\n",
						i + 1, (int)left_speed, (int)right_speed,
						(int)speed_difference);

				l0 = l1;
				r0 = r1;
				start = end;
			}

			motion_pidStop();
			tm_printf((UB*)"[S2-P09] Test completed\n");
		}
		#endif

		/* ============================================================
		* S2-DM01: Encoder-Based Distance Movement Test (FR6)
		*
		* Change '#if 0' to '#if 1' to enable
		* ============================================================ */
		#if 0
		{
			SYSTIM start, end;
			uint64_t start_ms, end_ms;
			float dt;
			int completed = 0;

			tm_printf((UB*)"[S2-D01] Move forward 30 cm test\n");
			tk_dly_tsk(5000);

			motion_pidInit();
			motion_pidMoveDistance(20.0f, 15.0f);

			/* Start movement */
			motion_setWheelPWM(30, 30);

			tk_get_tim(&start);

			for (int i = 0; i < 300; i++) {
				tk_dly_tsk(10);

				tk_get_tim(&end);

				start_ms = ((uint64_t)(uint32_t)start.hi << 32)
						| (uint32_t)start.lo;
				end_ms = ((uint64_t)(uint32_t)end.hi << 32)
					| (uint32_t)end.lo;

				if (end_ms <= start_ms) {
					break;
				}

				dt = (float)(end_ms - start_ms) / 1000.0f;

				motion_pidUpdate(dt);

				if (motion_pidIsComplete()) {
					completed = 1;

					tm_printf((UB*)"[S2-D01] Left encoder: %d cm\n",
							(int)encoder_getLeftDistance());

					tm_printf((UB*)"[S2-D01] Right encoder: %d cm\n",
							(int)encoder_getRightDistance());

					break;
				}

				start = end;
			}

			/* Always stop motors after the test */
			motion_pidStop();

			tk_dly_tsk(500);

			float left = encoder_getLeftDistance();
			float right = encoder_getRightDistance();

			tm_printf((UB*)"[S2-D01] Final left: %d.%02d cm\n",
					(int)left, ((int)(left * 100)) % 100);

			tm_printf((UB*)"[S2-D01] Final right: %d.%02d cm\n",
					(int)right, ((int)(right * 100)) % 100);

			tm_printf((UB*)"[S2-D01] Completed=%d\n", completed);
			tm_printf((UB*)"[S2-D01] Test finished\n");
		}
		#endif

		/* ============================================================
		* S2-T01: Encoder-Based 90 Degree Turning Test (FR6)
		*
		* Change '#if 0' to '#if 1' to enable
		* ============================================================ */
		#if 0
		{
			// tm_printf((UB*)"[S2-T01] Starting 90-degree left turn\n");
			// tm_printf((UB*)"[S2-T01] Starting 90-degree right turn\n");
			// tm_printf((UB*)"[S2-T01] Starting 180-degree right turn\n");
			tm_printf((UB*)"[S2-T01] Starting 180-degree left turn\n");

			tk_dly_tsk(5000);

			motion_pidInit();

			float left_start = encoder_getLeftDistance();
			float right_start = encoder_getRightDistance();

			/* 90° turn test */
			// motion_pidTurnLeft(90.0f);
			// motion_pidTurnRight(90.0f);

			/* 180° turn test */
			// motion_pidTurnRight(180.0f);
			motion_pidTurnLeft(180.0f);
			
			int completed = 0;

			for (int i = 0; i < 200; i++) {
				tk_dly_tsk(10);
				motion_pidUpdate(0.02f);

				if (motion_pidIsComplete()) {
					completed = 1;
					break;
				}
			}

			motion_pidStop();

			float left_travel =
    		encoder_getLeftDistance() - left_start;

			float right_travel =
				encoder_getRightDistance() - right_start;

			tm_printf((UB*)"[S2-T01] Left travel: %ld mm\n",
          			(long)(left_travel * 10.0f));

			tm_printf((UB*)"[S2-T01] Right travel: %ld mm\n",
					(long)(right_travel * 10.0f));

			tm_printf((UB*)"[S2-T01] Completed=%d\n", completed);
			tm_printf((UB*)"[S2-T01] Test finished\n");
		}
		#endif

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
