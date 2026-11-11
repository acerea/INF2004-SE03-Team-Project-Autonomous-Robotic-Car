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

/* VC-to-motion requests and motion completion notification. */
EXPORT ID mbf_motion_req;
EXPORT ID flg_motion;

/* Semaphore used to wake the motion task. */
static ID motion_sem;

/* Only the motion task changes these values. */
static float requested_speed_cm_s = 0.0f;
static int awaiting_motion_done = 0;

/* Process at most one full queue per cycle. */
#define MOTION_REQUESTS_PER_CYCLE 8u

/* Initialise hardware, controllers and shared objects. */
EXPORT ER subsystem2_init(void)
{
	T_CSEM	csem = {0};
	T_CMPF  cmpf = {0};
    T_CMBF  cmbf = {0};
    T_CFLG  cflg = {0};
    
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

    /* Create the VC request buffer.
    * Each message contains exactly two bytes.
    * 64 bytes allows eight queued requests, including kernel overhead
    * on this RP2040 port.
    */
    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz = 64;
    cmbf.maxmsz = MOTION_REQUEST_BYTES;

    mbf_motion_req = tk_cre_mbf(&cmbf);
    if (mbf_motion_req < E_OK) {
        return mbf_motion_req;
    }

    /* Create the completion flag, initially clear.
    * Motion sets MOTION_DONE; the VC clears it after reading.
    */
    cflg.flgatr = TA_TFIFO | TA_WSGL;
    cflg.iflgptn = 0;

    flg_motion = tk_cre_flg(&cflg);
    if (flg_motion < E_OK) {
        return flg_motion;
    }

    return E_OK;
}

/* Cyclic handler: signal the motion task without performing PID work here. */
static void motion_tick(void *exinf)
{
    (void)exinf;

    tk_sig_sem(motion_sem, 1);
}

static void motion_handleRequest(uint8_t command, uint16_t argument)
{
    switch (command) {
    case MOTION_CMD_STOP:
        motion_pidStop();
        requested_speed_cm_s = 0.0f;
        awaiting_motion_done = 0;
        break;

    case MOTION_CMD_SET_SPEED:
        /* Clamp requests to the agreed command limit. */
        if (argument > MOTION_MAX_SPEED_CM_S) {
            argument = MOTION_MAX_SPEED_CM_S;
        }

        requested_speed_cm_s = (float)argument;
        motion_pidChangeSpeed(requested_speed_cm_s);

        if (argument == 0u) {
            awaiting_motion_done = 0;
        }
        break;

    case MOTION_CMD_MOVE_DISTANCE:
        if (argument > 0u) {
            float speed = requested_speed_cm_s;

            if (speed <= 0.0f) {
                speed = (float)MOTION_DEFAULT_SPEED_CM_S;
            }

            motion_pidStop();
            motion_pidMoveDistance((float)argument, speed);
            awaiting_motion_done = 1;
        }
        break;

    case MOTION_CMD_TURN_LEFT:
        if (argument > 0u) {
            motion_pidStop();
            motion_pidTurnLeft((float)argument);
            awaiting_motion_done = 1;
        }
        break;

    case MOTION_CMD_TURN_RIGHT:
        if (argument > 0u) {
            motion_pidStop();
            motion_pidTurnRight((float)argument);
            awaiting_motion_done = 1;
        }
        break;

    default:
        /* Ignore reserved commands. */
        break;
    }
}

static void motion_receiveRequests(void)
{
    uint16_t requests[MOTION_REQUESTS_PER_CYCLE];
    UINT count = 0;
    int stop_received = 0;

    /* Read without blocking. A finite limit prevents starving PID. */
    for (UINT i = 0; i < MOTION_REQUESTS_PER_CYCLE; i++) {
        uint8_t bytes[MOTION_REQUEST_BYTES];

        INT size = tk_rcv_mbf(mbf_motion_req, bytes, TMO_POL);

        if (size < E_OK) {
            break;
        }

        if (size != (INT)MOTION_REQUEST_BYTES) {
            continue;
        }

        uint16_t request = motion_decodeRequest(bytes);

        requests[count++] = request;

        if (motion_unpackCommand(request) == MOTION_CMD_STOP) {
            stop_received = 1;

            /* Apply STOP immediately when encountered. */
            motion_handleRequest(MOTION_CMD_STOP, 0u);
        }
    }

    /* STOP overrides other requests read in this batch. */
    if (stop_received) {
        return;
    }

    for (UINT i = 0; i < count; i++) {
        motion_handleRequest(
            motion_unpackCommand(requests[i]),
            motion_unpackArgument(requests[i])
        );
    }
}

/* Run PID updates and temporarily report measured wake-up intervals. */
static void motion_control_task(INT stacd, void *exinf)
{
    SYSTIM previous_time;
    SYSTIM current_time;

    (void)stacd;
    (void)exinf;

    if (tk_get_otm(&previous_time) != E_OK) {
        tk_ext_tsk();
    }

    while (1) {
        /* Wait for the periodic signal. */
        if (tk_wai_sem(motion_sem, 1, TMO_FEVR) != E_OK) {
            continue;
        }

        motion_receiveRequests();

        if (tk_get_otm(&current_time) != E_OK) {
            continue;
        }

        UW elapsed_ms = (UW)(current_time.lo - previous_time.lo);
        previous_time = current_time;

        if (elapsed_ms > 0u) {
            motion_pidUpdate(elapsed_ms / 1000.0f);
        }

        /* Signal each successful distance/turn completion once.
        * Only the VC clears MOTION_DONE.
        */
        if (awaiting_motion_done && motion_pidIsComplete()) {
            if (tk_set_flg(flg_motion, MOTION_DONE) == E_OK) {
                awaiting_motion_done = 0;
            }
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