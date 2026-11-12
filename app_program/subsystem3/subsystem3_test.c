/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem3/TEST_LOG.md.
 *
 *	S3-U02..U13 test the pure algorithms (ir_algo.c) with synthetic data,
 *	so they pass or fail the same way on any board.  S3-U14..U16 check
 *	the live interfaces.
 */

#include <stdint.h>
#include "subsystem3.h"
#include "ir_algo.h"
#include "../test_report.h"

#if APP_TEST

/* Element widths (ms) for one Code 39 character: wide = 3 x narrow. */
LOCAL void widths_of(uint16_t pattern, uint16_t narrow, uint16_t w[9])
{
	UINT i;

	for (i = 0u; i < 9u; i++) {
		w[i] = (pattern & (1u << (8u - i))) ? (uint16_t)(3u * narrow)
						     : narrow;
	}
}

LOCAL BOOL str_is(const char *a, const char *b)
{
	while ((*a != '\0') && (*a == *b)) {
		a++;
		b++;
	}
	return (*a == *b) ? TRUE : FALSE;
}

EXPORT void subsystem3_test(void)
{
	ir_cal_t	cal;
	uint16_t	w[9];
	uint16_t	pats[4];
	char		word[12];
	int16_t		pos = 0;
	ir_snapshot_t	snap;
	T_RMBF		rmbf;
	UINT		i;
	BOOL		ok;

	TEST_INIT();

	/* S3-U02: calibration keeps learned levels with enough contrast,
	   falls back to the defaults without it */
	ir_cal_set(&cal, 300u, 3000u);
	ok = (cal.white == 300u) && (cal.black == 3000u);
	ir_cal_set(&cal, 1000u, 1200u);
	TEST_CHECK("S3-U02", ok && cal.white == IR_DEFAULT_WHITE &&
		   cal.black == IR_DEFAULT_BLACK,
		   "calibration: learned levels or safe defaults");

	/* S3-U03: darkness is 0 at white, 1000 at black, linear between */
	ir_cal_set(&cal, 500u, 2500u);
	TEST_CHECK("S3-U03", ir_darkness(400u, &cal) == 0u &&
		   ir_darkness(1500u, &cal) == 500u &&
		   ir_darkness(3000u, &cal) == 1000u,
		   "darkness normalisation 0..1000");

	/* S3-U04: hysteresis: the 40-60 % band keeps the old reading */
	TEST_CHECK("S3-U04", ir_is_black(1700u, FALSE, &cal) &&
		   !ir_is_black(1300u, TRUE, &cal) &&
		   ir_is_black(1500u, TRUE, &cal) &&
		   !ir_is_black(1500u, FALSE, &cal),
		   "black/white hysteresis");

	/* S3-U05: line position centred / left / right / half right */
	ok = ir_line_position(0u, 1000u, 0u, &pos) && (pos == 0);
	ok = ok && ir_line_position(1000u, 0u, 0u, &pos) && (pos == -1000);
	ok = ok && ir_line_position(0u, 0u, 1000u, &pos) && (pos == 1000);
	ok = ok && ir_line_position(0u, 1000u, 1000u, &pos) && (pos == 500);
	TEST_CHECK("S3-U05", ok, "line position estimate -1000..1000");

	/* S3-U06: all white is "no line" */
	TEST_CHECK("S3-U06", !ir_line_position(50u, 100u, 50u, &pos),
		   "all-white reading reports line lost");

	/* S3-U07: every letter decodes from ideal widths at 3 speeds */
	ok = TRUE;
	for (i = 0u; i < 26u; i++) {
		static const uint16_t letters[26] = {
			0x109u, 0x049u, 0x148u, 0x019u, 0x118u, 0x058u,
			0x00Du, 0x10Cu, 0x04Cu, 0x01Cu, 0x103u, 0x043u,
			0x142u, 0x013u, 0x112u, 0x052u, 0x007u, 0x106u,
			0x046u, 0x016u, 0x181u, 0x0C1u, 0x1C0u, 0x091u,
			0x190u, 0x0D0u };
		uint16_t n;

		for (n = 10u; n <= 40u; n += 15u) {
			widths_of(letters[i], n, w);
			if (ir_c39_lookup(ir_c39_pattern(w)) != (char)('A' + i)) {
				ok = FALSE;
			}
		}
	}
	TEST_CHECK("S3-U07", ok, "A-Z decode at narrow = 10, 25, 40 ms");

	/* S3-U08: +/-30 % timing jitter still decodes ('A') */
	widths_of(0x109u, 20u, w);
	w[0] = 60u * 13u / 10u;	w[1] = 14u;	w[2] = 26u;
	w[4] = 14u;		w[5] = 60u * 7u / 10u;
	TEST_CHECK("S3-U08", ir_c39_lookup(ir_c39_pattern(w)) == 'A',
		   "decode tolerates +/-30 % element jitter");

	/* S3-U09: elements all the same width are rejected, not guessed */
	for (i = 0u; i < 9u; i++) {
		w[i] = 20u;
	}
	TEST_CHECK("S3-U09", ir_c39_lookup(ir_c39_pattern(w)) == IR_C39_BAD,
		   "no wide/narrow contrast -> unreadable");

	/* S3-U10: "*A*" read forwards gives "A" */
	pats[0] = IR_C39_STAR; pats[1] = 0x109u; pats[2] = IR_C39_STAR;
	TEST_CHECK("S3-U10",
		   ir_c39_decode_word(pats, 3u, word, sizeof(word)) == 1u &&
		   str_is(word, "A"),
		   "forward read of *A* decodes A");

	/* S3-U11: "*B*" read backwards (robot from the other side) */
	pats[0] = ir_c39_reverse(IR_C39_STAR);
	pats[1] = ir_c39_reverse(0x049u);
	pats[2] = ir_c39_reverse(IR_C39_STAR);
	TEST_CHECK("S3-U11",
		   ir_c39_decode_word(pats, 3u, word, sizeof(word)) == 1u &&
		   str_is(word, "B"),
		   "backward read of *B* decodes B");

	/* S3-U12: command table (write-up example A/B/C/D) */
	TEST_CHECK("S3-U12", ir_nav_command("A") == IR_NAV_LEFT &&
		   ir_nav_command("B") == IR_NAV_RIGHT &&
		   ir_nav_command("C") == IR_NAV_STRAIGHT &&
		   ir_nav_command("D") == IR_NAV_UTURN &&
		   ir_nav_command("?") == IR_NAV_NONE &&
		   ir_nav_command("") == IR_NAV_NONE,
		   "barcode word -> navigation command");

	/* S3-U13: IF-15 byte packs letter (5 bits) and command (3 bits) */
	TEST_CHECK("S3-U13",
		   s3_nav_pack(25u, IR_NAV_UTURN) == 0x99u &&
		   s3_nav_letter_of(0x99u) == 25u &&
		   s3_nav_cmd_of(0x99u) == IR_NAV_UTURN &&
		   ir_letter_code('A') == 0u && ir_letter_code('?') == 31u,
		   "nav byte packing and letter codes");

	/* S3-U14: IF-15 message buffer exists with 1-byte messages */
	TEST_CHECK("S3-U14", tk_ref_mbf(mbf_s3_nav, &rmbf) == E_OK &&
		   rmbf.maxmsz == (INT)S3_NAV_MSG_BYTES,
		   "mbf_s3_nav exists, 1-byte messages");

	/* S3-U15: the IR task is running and has left calibration
	   (the test run starts 3 s after boot; calibration takes 0.5 s) */
	ir_get_telemetry(&snap);
	TEST_CHECK("S3-U15", snap.state != IR_CALI && snap.timestamp_ms != 0u,
		   "ir_sensor_task running and calibrated");

	/* S3-U16: snapshot fields stay within their bit ranges */
	TEST_CHECK("S3-U16", snap.line_pos >= -IR_POS_MAX &&
		   snap.line_pos <= IR_POS_MAX && snap.nav_cmd <= IR_NAV_UTURN,
		   "snapshot values within range");

	/* S3-U17: the snapshot fits in 8 bytes (timestamp + one word) */
	TEST_CHECK("S3-U17", sizeof(ir_snapshot_t) == 8u,
		   "snapshot is bit-packed into 8 bytes");

	TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */