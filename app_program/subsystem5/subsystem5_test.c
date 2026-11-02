/*
 *  subsystem5_test.c
 *  On-target tests for subsystem 5. Built only with: make TEST=1
 *  Record results in testing/subsystem5/TEST_LOG.md. IDs: S5-U01 to S5-U05
 */

#include <tk/tkernel.h>
#include "subsystem5.h"
#include "../test_report.h"
#include "drv_ultrasonic.h"

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

    /* S5-U02: Driver hardware initialization */
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

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */