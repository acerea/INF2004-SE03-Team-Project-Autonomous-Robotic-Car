/*
 *	subsystem2.c
 *	Subsystem 2 -- Producer: creates its objects and starts its task.
 *
 *	APIs shown: tk_cre_sem, tk_cre_mpf
 */

#include <stdint.h>

#include "motion_pid.h"
#include "pid_control.h"
#include "wheel_encoder.h"
#include "motion_control.h"
#include "subsystem2.h"

EXPORT ID	sem_slots;	/* IF-02, see subsystem2.h */
EXPORT ID	mpf_samples;

EXPORT ER subsystem2_init(void)
{
	motion_init(); /* Initialise motor control */
	encoder_init(); /* Initialise wheel encoders */
	motion_pidInit(); /* Initialise PID controller */

	T_CSEM	csem = {0};
	T_CMPF	cmpf = {0};

	/* Semaphore: a counter of free "slots".  The producer takes one
	   (tk_wai_sem) before each sample; the consumer gives one back
	   (tk_sig_sem).  It never exceeds the number of memory blocks, so
	   the producer can never run out of memory. */
	csem.sematr  = TA_TFIFO;	/* waiting tasks queue in FIFO order */
	csem.isemcnt = S2_POOL_COUNT;	/* initial count */
	csem.maxsem  = S2_POOL_COUNT;	/* maximum count */
	sem_slots = tk_cre_sem(&csem);
	if (sem_slots < E_OK) {
		return sem_slots;
	}

	/* Fixed-size memory pool: S2_POOL_COUNT blocks, each exactly one
	   SAMPLE_MSG.  Faster and more predictable than malloc(). */
	cmpf.mpfatr = TA_TFIFO | TA_RNG3;
	cmpf.mpfcnt = S2_POOL_COUNT;		/* number of blocks */
	cmpf.blfsz  = sizeof(SAMPLE_MSG);	/* size of one block */
	mpf_samples = tk_cre_mpf(&cmpf);
	return (mpf_samples < E_OK) ? mpf_samples : E_OK;
}

static ID motion_sem;

/* Signal the motion task every 20 ms */
static void motion_tick(void *exinf)
{
    (void)exinf;
    tk_sig_sem(motion_sem, 1);
}

/* Periodically update encoder-based motion control */
static void motion_control_task(INT stacd, void *exinf)
{
    SYSTIM previous_time;
    SYSTIM current_time;

    (void)stacd;
    (void)exinf;

    if (tk_get_otm(&previous_time) != E_OK) {
        return;
    }

    while (1) {
        /* Wait for the next 20 ms signal */
        if (tk_wai_sem(motion_sem, 1, TMO_FEVR) != E_OK) {
            continue;
        }

        if (tk_get_otm(&current_time) != E_OK) {
            continue;
        }

        UW elapsed_ms =
            (UW)(current_time.lo - previous_time.lo);

        previous_time = current_time;

        if (elapsed_ms > 0u) {
            motion_pidUpdate(elapsed_ms / 1000.0f);
        }
    }
}

EXPORT ER subsystem2_start(void)
{
    ID tid;
    ID cycid;
    ER er;

    T_CSEM csem = {0};
    T_CCYC ccyc = {0};

    /* Existing sample producer task */
    tid = app_start_task(producer_task, S2_PRI_PRODUCER,
                         TP_PRC2, "producer_task");
    if (tid < E_OK) {
        return tid;
    }

	 /* Create semaphore for periodic motion updates */
    csem.sematr = TA_TFIFO;
    csem.isemcnt = 0;
    csem.maxsem = 1;

    motion_sem = tk_cre_sem(&csem);

    if (motion_sem < E_OK) {
        return motion_sem;
    }

    /* Start periodic PID updates under micro T-Kernel */
    tid = app_start_task(motion_control_task, S2_PRI_MOTION,
                         TP_PRC2, "motion_control_task");
    if (tid < E_OK) {
        return tid;
    }

    /* Configure the 20 ms cyclic handler */
    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = (FP)motion_tick;
    ccyc.cyctim = S2_MOTION_MS;
    ccyc.cycphs = S2_MOTION_MS;

    cycid = tk_cre_cyc(&ccyc);

    if (cycid < E_OK) {
        return cycid;
    }

    er = tk_sta_cyc(cycid);

    return er;
}