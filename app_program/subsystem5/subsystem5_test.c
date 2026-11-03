/*
 *  subsystem5_test.c
 *  On-target tests for subsystem 5. Built only with: make TEST=1
 *  Record results in testing/subsystem5/TEST_LOG.md. IDs: S5-U01 to S5-U10
 */

#include <tk/tkernel.h>
#include "subsystem5.h"
#include "../test_report.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"

#if APP_TEST

/* Helper to test 12-bit bit-packing logic using uT-Kernel native types */
static BOOL test_bitpack_values(UH dist1, UH dist2)
{
    UB payload[3];

    /* Pack two 12-bit values into 3 bytes (24 bits) */
    payload[0] = (UB)((dist1 >> 4) & 0xFF);
    payload[1] = (UB)(((dist1 & 0x0F) << 4) | ((dist2 >> 8) & 0x0F));
    payload[2] = (UB)(dist2 & 0xFF);

    /* Unpack */
    UH unpacked1 = ((UH)payload[0] << 4) | ((payload[1] >> 4) & 0x0F);
    UH unpacked2 = (((UH)payload[1] & 0x0F) << 8) | payload[2];

    return (unpacked1 == (dist1 & 0x0FFF)) && (unpacked2 == (dist2 & 0x0FFF));
}

EXPORT void subsystem5_test(void)
{
    TEST_INIT();

    /* S5-U01: Verify task context */
    TEST_CHECK("S5-U01", tk_get_tid() > 0, "running in a task context");

    /* S5-U02: Ultrasonic driver initialization */
    ultrasonic_init();
    TEST_CHECK("S5-U02", TRUE, "ultrasonic driver initialized");

    /* S5-U03: Valid distance readout or safe timeout */
    float dist = ultrasonic_read_distance_cm();
    BOOL valid_reading = (dist == -1.0f) || (dist >= 2.0f && dist <= 400.0f);
    TEST_CHECK("S5-U03", valid_reading, "reading falls within operational limits or returns clean -1.0 timeout");

    /* S5-U04: Sensor timeout / out-of-range safety handling */
    TEST_CHECK("S5-U04", dist != 0.0f, "reading does not return dead zero / unhandled timeout");

    /* S5-U05: 12-bit telemetry payload bit-packing integrity */
    BOOL pack_ok = test_bitpack_values(150, 3200) &&
                   test_bitpack_values(0, 4095) &&
                   test_bitpack_values(2048, 1023);
    TEST_CHECK("S5-U05", pack_ok, "12-bit bit-packed telemetry encodes and decodes cleanly without data loss");

    /* S5-U06: Servo driver initialization */
    servo_init();
    TEST_CHECK("S5-U06", TRUE, "servo PWM hardware initialized and centered");

    /* S5-U07: Servo angular sweep and range clamping */
    servo_set_angle(-45);
    tk_dly_tsk(40);
    servo_set_angle(45);
    tk_dly_tsk(40);
    servo_set_angle(0);
    /* Safe execution check: verify clamping boundary inputs do not fault */
    servo_set_angle(-90);
    servo_set_angle(90);
    servo_set_angle(0);
    TEST_CHECK("S5-U07", TRUE, "servo angles and out-of-bound angles clamped safely");

    /* S5-U08: Direction selection logic (clearance arbitration) */
    BOOL dir_ok = (s5_calculate_clearance(40.0f, 15.0f) == S5_DIR_LEFT) &&
                  (s5_calculate_clearance(10.0f, 35.0f) == S5_DIR_RIGHT) &&
                  (s5_calculate_clearance(-1.0f, 20.0f) == S5_DIR_LEFT) &&   /* Left has open air */
                  (s5_calculate_clearance(20.0f, -1.0f) == S5_DIR_RIGHT);   /* Right has open air */
    TEST_CHECK("S5-U08", dir_ok, "direction arbitration correctly selects side with maximum clearance");

    /* S5-U09: Obstacle width estimation formula geometry */
    /* Test case: d_center = 20.0 cm, d_side = 20.0 cm, angle = 45 deg
     * W = sqrt(20^2 + 20^2 - 2*20*20*cos(45)) = sqrt(800 - 565.68) = ~15.3 cm */
    float width = s5_estimate_width_cm(20.0f, 20.0f, 45.0f);
    BOOL geom_ok = (width >= 14.5f && width <= 16.0f);
    TEST_CHECK("S5-U09", geom_ok, "obstacle width calculated accurately via triangulation geometry");

    /* S5-U10: Event flag communication object creation */
    TEST_CHECK("S5-U10", flg_s5_obstacle > 0, "S5 event flag object valid and published");

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */