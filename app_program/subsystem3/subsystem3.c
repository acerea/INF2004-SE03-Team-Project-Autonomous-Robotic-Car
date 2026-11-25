/*
 *	subsystem3.c
 *	Subsystem 3 -- IR line following and barcode decoding: creates its
 *	objects and starts its task.
 */

#include "subsystem3.h"
#include "ir_sense.h"

EXPORT ID		mbx_samples;	/* IF-01 (legacy example) */
EXPORT ID		mtx_stats;	/* IF-04 (legacy example) */
EXPORT APP_STATS	app_stats;

EXPORT ER subsystem3_init(void)
{
	T_CMBX	cmbx = {0};
	T_CMTX	cmtx = {0};
	ER	er;

	/* IR sensing: mutex, IF-15 message buffer, ADC "adca" */
	er = ir_sense_init();
	if (er < E_OK) {
		return er;
	}

	/* Legacy example objects, still read by subsystem 4's monitor. */
	cmbx.mbxatr = TA_TFIFO | TA_MFIFO;
	mbx_samples = tk_cre_mbx(&cmbx);
	if (mbx_samples < E_OK) {
		return mbx_samples;
	}

	cmtx.mtxatr = TA_INHERIT;
	mtx_stats = tk_cre_mtx(&cmtx);
	return (mtx_stats < E_OK) ? mtx_stats : E_OK;
}

EXPORT ER subsystem3_start(void)
{
	/* The legacy consumer_task is not started: subsystem 2 no longer
	   runs the producer, so it would only wait forever on an empty
	   mailbox and hold a 2 KB stack. */
	return ir_sense_start();
}