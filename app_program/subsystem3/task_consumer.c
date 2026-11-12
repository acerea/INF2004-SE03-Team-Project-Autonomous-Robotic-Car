/*
 *	task_consumer.c
 *	Subsystem 3 -- the consumer task.
 *
 *	APIs shown: tk_rcv_mbx, tk_rel_mpf, tk_sig_sem, tk_loc_mtx,
 *	            tk_unl_mtx, tk_snd_mbf, tk_set_flg
 */

#include "subsystem3.h"
#include "../subsystem2/subsystem2.h"	/* IF-01/02: SAMPLE_MSG, pool, slots */
#include "../subsystem4/subsystem4.h"	/* IF-03: mbf_summary, flg_events */

#include "../app.h"

IMPORT ID sem_slots;
IMPORT ID mpf_samples;
IMPORT ID mpx_samples;

static void
s3_publish_barcode(char letter, uint32_t command)
{
	uint8_t      frame[PROTO_MAX_FRAME_BYTES];
	proto_bits_t bits;
	uint32_t     code = PROTO_BC_LETTER_BAD;    // Not A-Z: unreadable.

	if ((letter >= 'A') && (letter <= 'Z'))
	{
		code = (uint32_t)(letter - 'A');       // 'A' = 0 ... 'Z' = 25.
	}

	proto_init(&bits, frame, sizeof(frame));
	proto_begin(&bits, PROTO_TYPE_BARCODE);
	proto_put(&bits, code, PROTO_BC_LETTER_BITS);
	proto_put(&bits, command, PROTO_BC_CMD_BITS);
	(void)s1_publish(frame, proto_end(&bits), false);
}

EXPORT void consumer_task(INT stacd, void *exinf)
{
	SAMPLE_MSG	*msg;
	UB		text[S4_SUMMARY_MAX];	/* tm_sprintf adds a '\0' */
	INT		len;
	UW		count;
	ER		er;

	(void)stacd; (void)exinf;

	while (1) {
		/* 1. Wait for the next sample (TMO_FEVR = wait forever) */
		er = tk_rcv_mbx(mbx_samples, (T_MSG **)&msg, TMO_FEVR);
		if (er < E_OK) {
			CHECK(er, "tk_rcv_mbx");
			continue;
		}

		/* 2. Update the shared statistics.  Subsystem 4 reads them
		      too, so lock the mutex around every access (IF-04). */
		tk_loc_mtx(mtx_stats, TMO_FEVR);
		if (app_stats.received == 0 || msg->value < app_stats.min) {
			app_stats.min = msg->value;
		}
		if (app_stats.received == 0 || msg->value > app_stats.max) {
			app_stats.max = msg->value;
		}
		app_stats.sum += msg->value;
		count = ++app_stats.received;
		tk_unl_mtx(mtx_stats);

		/* 3. Every S3_BATCH_SIZE samples, send a text line to subsystem
		      4 through its message buffer (the text is copied), then
		      set FLG_BATCH to wake it up (IF-03). */
		if (count % S3_BATCH_SIZE == 0) {
			/* tm_sprintf: like sprintf, returns the length.
			   The longest line here is ~35 bytes < S4_SUMMARY_MAX. */
			len = tm_sprintf(text, (const UB*)"n=%u last=%d from core %d",
					 count, msg->value, msg->prc - 1);

			/* TMO_POL: if the buffer is full, drop the line rather
			   than block the data path. */
			er = tk_snd_mbf(mbf_summary, text, len, TMO_POL);
			if (er == E_OK) {
				tk_set_flg(flg_events, FLG_BATCH);
			}
		}

		/* 4. Done with the sample (IF-02): return its memory block to
		      the pool, then the slot so the producer may go on. */
		tk_rel_mpf(mpf_samples, msg);
		tk_sig_sem(sem_slots, 1);
	}
}


