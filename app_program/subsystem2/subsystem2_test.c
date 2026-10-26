/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.
 */

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
		* Change '#if 0' to '#if 1' to enable the test
		* Ensure the robot car is stationary and connect to serial monitor to see the output.
		* ===================================================================================== */
		
		#if 0

		tm_printf((UB*)"[S2-M01] Starting motor test\n");

		/* Allow time to connect the serial monitor */
		tk_dly_tsk(5000);

		/* Move both wheels forward at 40% PWM and stop after 300ms */
		tm_printf((UB*)"[S2-M01] Moving forward now\n");
		motion_moveForward(40);
		tk_dly_tsk(300);
		motion_stop();

		/* Move both wheels backward at 40% PWM and stop after 300ms */
		tm_printf((UB*)"[S2-M01] Moving backward now\n");
		motion_moveBackward(40);
		tk_dly_tsk(300);
		motion_stop();

		/* Turn left at 40% PWM and stop after 300ms */
		tm_printf((UB*)"[S2-M01] Turning left now\n");
		motion_turnLeft(40);
		tk_dly_tsk(300);
		motion_stop();

		/* Turn right at 40% PWM and stop after 300ms */
		tm_printf((UB*)"[S2-M01] Turning right now\n");
		motion_turnRight(40);
		tk_dly_tsk(300);
		motion_stop();

		tm_printf((UB*)"[S2-M01] Motor test completed\n");

		#endif


		/* ====================================================================================
		* S2-E01: Wheel Encoder Test
		*
		* Tests:
		* - Encoder Pulse Counting
		* - Wheel Distance Estimation (cm)
		* - Speed Estimation (cm/s)
		*
		* Change '#if 0' to '#if 1' to enable the test
		* ===================================================================================== */
		
		#if 0

		tm_printf((UB*)"[S2-E01] Starting wheel encoder test\n");

		/* Allow time to connect the serial monitor */
		tk_dly_tsk(10000);

		/* Initialise previous encoder counts */
        int32_t prev_leftCount = encoder_getLeftCount();
        int32_t prev_rightCount = encoder_getRightCount();

		/* Monitor encoder readings */
		for (int i = 0; i < 300; i++) {

			/* Read current encoder counts */
			int32_t current_leftCount = encoder_getLeftCount();
			int32_t current_rightCount = encoder_getRightCount();
				
			/* Display encoder counts */
			tm_printf((UB*)"[S2-ENC] Left count: %ld | Right count: %ld\n",
						(long)current_leftCount,
              			(long)current_rightCount);

			/* Calculate wheel distance (cm) */
			int left_mm = (int)(encoder_getLeftDistance() * 10.0f);
			int right_mm = (int)(encoder_getRightDistance() * 10.0f);

			/* Display distance (cm) */
			tm_printf((UB*)"Left distance: %d.%01d cm | Right distance: %d.%01d cm\n",
						left_mm / 10, left_mm % 10,
           				right_mm / 10, right_mm % 10);

			/* Calculate encoder count changes */
            int32_t leftCount_change = current_leftCount - prev_leftCount;
            int32_t rightCount_change = current_rightCount - prev_rightCount;

            /* Estimate wheel speed (cm/s) */
            float left_speed = encoder_calculateLeftSpeed(leftCount_change, 0.2f);
            float right_speed = encoder_calculateRightSpeed(rightCount_change, 0.2f);
            
            /* Update previous counts */
            prev_leftCount = current_leftCount;
            prev_rightCount = current_rightCount;

            /* Convert speed to cm/s */
            int left_speed10 = (int)(left_speed * 10.0f);
            int right_speed10 = (int)(right_speed * 10.0f);

            /* Display wheel speed */
            tm_printf((UB*)"Speed L: %d.%01d cm/s | R: %d.%01d cm/s\n",
                        left_speed10 / 10, left_speed10 % 10,
                        right_speed10 / 10, right_speed10 % 10);

			tk_dly_tsk(200);
		}
		
		tm_printf((UB*)"[S2-E01] Wheel encoder test completed\n");

		#endif 

		/* ====================================================================================
		* S2-E02: Motor-driven Encoder Test
		*
		* Tests:
		* - Encoder counting during motor operation
		*
		* Change '#if 0' to '#if 1' to enable the test
		* ===================================================================================== */
		
		#if 0

		tm_printf((UB*)"[S2-E02] Starting motor-driven encoder test\n");

		/* Allow time to connect the serial monitor */
		tk_dly_tsk(5000);

		/* Read counts before movement */
		int32_t left_before = encoder_getLeftCount();
		int32_t right_before = encoder_getRightCount();

		/* Run motors briefly */
		motion_moveForward(40);
		tk_dly_tsk(500);
		motion_stop();

		/* Read counts after movement */
		int32_t left_after = encoder_getLeftCount();
		int32_t right_after = encoder_getRightCount();

		tm_printf((UB*)"[S2-E02] Left count change: %ld\n",
				(long)(left_after - left_before));

		tm_printf((UB*)"[S2-E02] Right count change: %ld\n",
				(long)(right_after - right_before));

		tm_printf((UB*)"[S2-E02] Test completed\n");

		#endif 

		/* ====================================================================================
		* S2-D01: Motor-driven Encoder and Distance Test
		*
		* Tests:
		* - Encoder counting during motor operation
 		* - Encoder-based wheel distance estimation
		*
		* Change '#if 0' to '#if 1' to enable the test
		* ===================================================================================== */
		
		#if 0

		tm_printf((UB*)"[S2-ED01] Starting motor-driven distance test\n");

		/* Allow time to connect the serial monitor */
		tk_dly_tsk(5000);

		/* Read counts before movement */
		int32_t left_before = encoder_getLeftCount();
		int32_t right_before = encoder_getRightCount();

		/* Move forward at 40% PWM for 500 ms */
		motion_moveForward(40);
		tk_dly_tsk(200);
		motion_stop();

		/* Allow wheels to stop completely */
		tk_dly_tsk(500);

		/* Read counts after movement */
		int32_t left_after = encoder_getLeftCount();
		int32_t right_after = encoder_getRightCount();

		/* Calculate encoder count changes */
		int32_t left_change = left_after - left_before;
		int32_t right_change = right_after - right_before;

		/* Display encoder count changes */
		tm_printf((UB*)"[S2-ED01] Left count change: %ld\n",
				(long)left_change);

		tm_printf((UB*)"[S2-ED01] Right count change: %ld\n",
				(long)right_change);

		/* Calculate distance travelled during the test (cm) */
		float left_distance = ((float)left_change / 2523.0f)
							* (3.14159265f * 6.0f);

		float right_distance = ((float)right_change / 2535.0f)
							* (3.14159265f * 6.0f);

		/* Convert to tenths of a centimetre for serial printing */
		int left_distance10 = (int)(left_distance * 10.0f);
		int right_distance10 = (int)(right_distance * 10.0f);

		/* Display estimated wheel distances */
		tm_printf((UB*)"[S2-ED01] Left distance: %d.%01d cm\n",
				left_distance10 / 10,
				left_distance10 % 10);

		tm_printf((UB*)"[S2-ED01] Right distance: %d.%01d cm\n",
				right_distance10 / 10,
				right_distance10 % 10);

		tm_printf((UB*)"[S2-ED01] Test completed\n");

		#endif 

		/* ====================================================================================
		* S2-S01: Motor-driven Speed Test
		*
		* Tests:
		* - Wheel speed while motors are running
 		* - Wheel speed after motors stop
		*
		* Change '#if 0' to '#if 1' to enable the test
		* ===================================================================================== */
		
		#if 0

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
	
		#endif 

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
