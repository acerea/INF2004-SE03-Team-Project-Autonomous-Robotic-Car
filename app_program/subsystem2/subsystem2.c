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
#include "../subsystem1/subsystem1.h"

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
static uint32_t motion_mode = PROTO_MO_MODE_STOP;   /* For telemetry. */

/* MOTION telemetry (IF-10, type 2): every 200 ms moving, 1 s stopped. */
#define MOTION_TLM_MOVING_MS 200u
#define MOTION_TLM_IDLE_MS   1000u

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
    cmbf.maxmsz = S2_MOTION_REQUEST_BYTES;

    mbf_motion_req = tk_cre_mbf(&cmbf);
    if (mbf_motion_req < E_OK) {
        return mbf_motion_req;
    }

    /* Create the completion flag, initially clear.
    * Motion sets S2_MOTION_DONE; the VC clears it after reading.
    */
    cflg.flgatr = TA_TFIFO | TA_WSGL;
    cflg.iflgptn = 0;

    flg_motion = tk_cre_flg(&cflg);
    if (flg_motion < E_OK) {
        return flg_motion;
    }

    return E_OK;
}

EXPORT void s2_setLineCorrection(int line_pos)
{
    motion_pidSetLineCorrection(line_pos);
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
    case S2_MOTION_STOP:
        motion_pidStop();
        requested_speed_cm_s = 0.0f;
        awaiting_motion_done = 0;
        motion_mode = PROTO_MO_MODE_STOP;
        break;

    case S2_MOTION_SET_SPEED:
        /* Clamp requests to the agreed command limit. */
        if (argument > S2_MOTION_MAX_SPEED) {
            argument = S2_MOTION_MAX_SPEED;
        }

        requested_speed_cm_s = (float)argument;
        motion_pidChangeSpeed(requested_speed_cm_s);

        /* A speed change keeps an active distance move or turn. */
        if ((motion_mode != PROTO_MO_MODE_DISTANCE)
            && (motion_mode != PROTO_MO_MODE_TURN)) {
            motion_mode = PROTO_MO_MODE_SPEED;
        }

        if (argument == 0u) {
            awaiting_motion_done = 0;
        }
        break;

    case S2_MOTION_MOVE_DISTANCE:
        if (argument > 0u) {
            float speed = requested_speed_cm_s;

            if (speed <= 0.0f) {
                speed = (float)S2_MOTION_DEFAULT_SPEED;
            }

            motion_pidStop();
            motion_pidMoveDistance((float)argument, speed);
            awaiting_motion_done = 1;
            motion_mode = PROTO_MO_MODE_DISTANCE;
        }
        break;

    case S2_MOTION_TURN_LEFT:
        if (argument > 0u) {
            motion_pidStop();
            motion_pidTurnLeft((float)argument);
            awaiting_motion_done = 1;
            motion_mode = PROTO_MO_MODE_TURN;
        }
        break;

    case S2_MOTION_TURN_RIGHT:
        if (argument > 0u) {
            motion_pidStop();
            motion_pidTurnRight((float)argument);
            awaiting_motion_done = 1;
            motion_mode = PROTO_MO_MODE_TURN;
        }
        break;

    default:
        /* Ignore reserved commands. */
        break;
    }
}

static void motion_receiveRequests(void)
{
    /* Read without blocking. A finite limit prevents starving PID.
     *
     * Requests are applied strictly in the order the VC sent them. A STOP
     * still cancels everything queued before it, but a request sent AFTER
     * the STOP (e.g. STOP then SET_SPEED when a bypass finishes) must not
     * be thrown away: the VC only resends a request when it changes, so a
     * dropped one left the car standing still in FOLLOWING. */
    for (UINT i = 0; i < MOTION_REQUESTS_PER_CYCLE; i++) {
        uint8_t bytes[S2_MOTION_REQUEST_BYTES];

        INT size = tk_rcv_mbf(mbf_motion_req, bytes, TMO_POL);

        if (size < E_OK) {
            break;
        }

        if (size != (INT)S2_MOTION_REQUEST_BYTES) {
            continue;
        }

        uint16_t request = motion_decodeRequest(bytes);

        motion_handleRequest(
            motion_unpackCommand(request),
            motion_unpackArgument(request)
        );
    }
}

/* Speed in 0.1 cm/s from an encoder count change over elapsed_ms. */
static int32_t motion_tenths(float speed_cm_s)
{
    return (int32_t)((speed_cm_s >= 0.0f) ? (speed_cm_s * 10.0f + 0.5f)
                                          : (speed_cm_s * 10.0f - 0.5f));
}

/* Publish one MOTION frame when it is due. Never blocks. */
static void motion_publishTelemetry(UW now_ms)
{
    static UW last_ms = 0u;
    static int32_t last_left = 0;
    static int32_t last_right = 0;
    UW period = (motion_mode == PROTO_MO_MODE_STOP) ? MOTION_TLM_IDLE_MS
                                                    : MOTION_TLM_MOVING_MS;
    UW elapsed_ms = now_ms - last_ms;

    if (elapsed_ms < period) {
        return;
    }

    int32_t left = encoder_getLeftCount();
    int32_t right = encoder_getRightCount();
    float dt = (float)elapsed_ms / 1000.0f;
    float left_speed = encoder_calculateLeftSpeed(left - last_left, dt);
    float right_speed = encoder_calculateRightSpeed(right - last_right, dt);
    int left_pwm;
    int right_pwm;
    uint8_t frame[PROTO_MAX_FRAME_BYTES];
    proto_bits_t bits;

    last_ms = now_ms;
    last_left = left;
    last_right = right;
    motion_getWheelPWM(&left_pwm, &right_pwm);

    proto_init(&bits, frame, sizeof(frame));
    proto_begin(&bits, PROTO_TYPE_MOTION);
    proto_put(&bits, motion_mode, PROTO_MO_MODE_BITS);
    proto_put_sat(&bits, (uint32_t)requested_speed_cm_s, PROTO_MO_TARGET_BITS);
    proto_put_s(&bits, motion_tenths(left_speed), PROTO_MO_SPEED_BITS);
    proto_put_s(&bits, motion_tenths(right_speed), PROTO_MO_SPEED_BITS);
    proto_put_s(&bits, (int32_t)left_pwm, PROTO_MO_PWM_BITS);
    proto_put_s(&bits, (int32_t)right_pwm, PROTO_MO_PWM_BITS);
    (void)s1_publish(frame, proto_end(&bits), false);
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
        * Only the VC clears S2_MOTION_DONE.
        */
        if (awaiting_motion_done && motion_pidIsComplete()) {
            if (tk_set_flg(flg_motion, S2_MOTION_DONE) == E_OK) {
                awaiting_motion_done = 0;
                motion_mode = PROTO_MO_MODE_STOP;
            }
        }

        motion_publishTelemetry((UW)current_time.lo);
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
