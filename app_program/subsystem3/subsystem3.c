/*
 *	subsystem3.c
 *	Subsystem 3 -- Consumer: creates its objects and starts its task.
 *
 *	APIs shown: tk_cre_mbx, tk_cre_mtx
 */

#include "subsystem3.h"
#include "ir_sense.h"

EXPORT ID		mbx_samples;	/* IF-01 */
EXPORT ID		mtx_stats;	/* IF-04 */
EXPORT APP_STATS	app_stats;

EXPORT buddy3_context_t ir_buddy_ctx; 
EXPORT ID ir_state_mutex; 

/*system integration updates for IF-01,02*/

/*end system integration updates for IF-01,02*/

/*system integration updates for IF-03,04*/

/*end system integration updates for IF-03,04*/

EXPORT ER subsystem3_init(void)
{

	T_CMBX	cmbx = {0};
	T_CMTX	cmtx = {0};

	ir_buddy_ctx.threshold_black = 0;  
	ir_buddy_ctx.threshold_white = 0;
	ir_buddy_ctx.edge_count = 0;
	ir_buddy_ctx.string_index = 0;
	ir_buddy_ctx.emergency_stop_active = false;
	ir_buddy_ctx.current_state.current_state = IR_CALI;

	ir_sense_init(); 

	/* Mailbox: passes a POINTER to a message (zero copy).  The message
	   memory must stay valid until the receiver is done with it, which
	   is why subsystem 2 takes it from its memory pool. */
	cmbx.mbxatr = TA_TFIFO | TA_MFIFO;	/* FIFO waiters, FIFO messages */
	mbx_samples = tk_cre_mbx(&cmbx);
	if (mbx_samples < E_OK) {
		return mbx_samples;
	}

	/* Mutex: only one task at a time may touch app_stats.  TA_INHERIT
	   (priority inheritance) avoids priority inversion: a low-priority
	   holder is temporarily raised to the waiter's priority. */
	cmtx.mtxatr = TA_INHERIT;
	mtx_stats = tk_cre_mtx(&cmtx);

	return (mtx_stats < E_OK) ? mtx_stats : E_OK;
}

EXPORT ER subsystem3_start(void)
{
	ir_sense_start();

	/* Pinned to core 0 (TP_PRC1): opposite core to the producer */
	ID tid = app_start_task(consumer_task, S3_PRI_CONSUMER, TP_PRC1,
				"consumer_task");
	return (tid < E_OK) ? tid : E_OK;
}
