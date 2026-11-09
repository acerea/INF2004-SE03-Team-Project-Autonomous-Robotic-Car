/*
 *	subsystem2.c
 *	Subsystem 2 -- Producer: creates its objects and starts its task.
 *
 *	APIs shown: tk_cre_sem, tk_cre_mpf
 */

#include <tm/tmonitor.h>

#include "motion_control.h"
#include "motion_pid.h"
#include "subsystem2.h"
#include "wheel_encoder.h"

EXPORT ID	sem_slots;	/* IF-02, see subsystem2.h */
EXPORT ID	mpf_samples;

/* Semaphore used to wake the motion task. */
static ID motion_sem;

/* Initialise hardware, controllers and shared objects. */
EXPORT ER subsystem2_init(void)
{
	T_CSEM	csem = {0};
	T_CMPF	cmpf = {0};
    
    motion_init();
	encoder_init();
	motion_pidInit();

	/* Legacy producer/consumer slot semaphore. */
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
    if (mpf_samples < E_OK) {
        return mpf_samples;
    }

    return E_OK;
}

/* Cyclic handler: signal the motion task without performing PID work here. */
static void motion_tick(void *exinf)
{
    (void)exinf;

    tk_sig_sem(motion_sem, 1);
}

/* Run PID updates and temporarily report measured wake-up intervals. */
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
        /* Wait for the periodic signal. */
        if (tk_wai_sem(motion_sem, 1, TMO_FEVR) != E_OK) {
            continue;
        }

        if (tk_get_otm(&current_time) != E_OK) {
            continue;
        }

        UW elapsed_ms = (UW)(current_time.lo - previous_time.lo);
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

    T_CSEM csem = {0};
    T_CCYC ccyc = {0};

    /* Legacy sample producer is intentionally not started. */

	/* Allow at most one pending motion signal. */
    csem.sematr = TA_TFIFO;
    csem.isemcnt = 0;
    csem.maxsem = 1;

    motion_sem = tk_cre_sem(&csem);
    if (motion_sem < E_OK) {
        return motion_sem;
    }

    tid = app_start_task(
        motion_control_task,
        S2_PRI_MOTION,
        TP_PRC2,
        "motion_control_task"
    );

    if (tid < E_OK) {
        return tid;
    }

    /* Configure the periodic trigger using S2_MOTION_MS. */
    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = (FP)motion_tick;
    ccyc.cyctim = S2_MOTION_MS;
    ccyc.cycphs = S2_MOTION_MS;

    cycid = tk_cre_cyc(&ccyc);
    if (cycid < E_OK) {
        return cycid;
    }

    return tk_sta_cyc(cycid);
}