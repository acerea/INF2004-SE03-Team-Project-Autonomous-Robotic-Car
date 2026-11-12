/** @file vehicle.c
 *
 * @brief Vehicle Controller (VC): mission state machine, stage 1.
 *
 * @par
 * Stage 1: START / STOP / ESTOP / RESET from the laptop, obstacle
 * slow-down and stop from subsystem 5, and stop when subsystem 4 reports
 * the car lifted or overturned (end of run). Turning (S3) and bypassing (S5)
 * are added in later stages. The VC polls its inputs every VC_TICK_MS,
 * because a task can wait on only one kernel object at a time, turns them
 * into events and handles each event in vc_dispatch() (switch on state).
 *
 * State table (ESTOP, STOP and LIFTED are accepted in every state):
 *   any        ESTOP              -> FAULT      motors stop
 *   not FAULT  STOP               -> STOPPED    motors stop
 *   driving    LIFTED             -> STOPPED    motors stop
 *   IDLE, STOPPED, FAULT  LIFTED  -> no change  motors stop
 *   IDLE       START              -> FOLLOWING  speed by obstacle level
 *   FOLLOWING  OBS_EMERGENCY      -> BLOCKED    motors stop
 *   FOLLOWING  OBS_SLOW / CLEAR   -> FOLLOWING  speed 10 / 20 cm/s
 *   BLOCKED    OBS_SLOW / CLEAR   -> FOLLOWING  speed 10 cm/s
 *   STOPPED    START              -> FOLLOWING  speed by obstacle level
 *   STOPPED    RESET              -> IDLE       motors stop
 *   FAULT      RESET              -> IDLE       motors stop
 *   Any other state/event pair is ignored (e.g. START in FAULT).
 *   START is also ignored while the car is lifted.
 */
#include <stdint.h>
#include <stdbool.h>
#include "vehicle.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"

// Motion requests go to subsystem 2's mbf_motion_req (2 bytes, see
// subsystem2.h). The VC never blocks on it: a request that does not fit is
// kept and sent again on the next tick.
#define VC_MOTION_NONE      (0xFFFFu)             // Nothing sent yet.

#define VC_REMOTE_EVENTS    (S1_FLG_ESTOP | S1_FLG_STOP | S1_FLG_CMD_READY)

// Obstacle level from subsystem 5, worst first.
#define VC_OBS_CLEAR        (0u)
#define VC_OBS_SLOW         (1u)
#define VC_OBS_EMERGENCY    (2u)

// Events: every input is turned into one of these and passed to
// vc_dispatch(), the only place where the mission state changes.
typedef uint8_t vc_event_t;
#define VC_EV_START         (0u)    // Remote START.
#define VC_EV_STOP          (1u)    // Remote STOP (flag from comms).
#define VC_EV_ESTOP         (2u)    // Remote ESTOP (flag from comms).
#define VC_EV_RESET         (3u)    // Remote RESET.
#define VC_EV_OBS_CLEAR     (4u)    // Obstacle level from S5, every tick.
#define VC_EV_OBS_SLOW      (5u)
#define VC_EV_OBS_EMERGENCY (6u)
#define VC_EV_LIFTED        (7u)    // Car lifted or overturned (S4), every tick.

// Lift detection is on once subsystem 4 publishes S4_FLAG_LIFTED (IF-13).
#ifdef S4_FLAG_LIFTED
#define VC_LIFT_DETECT      (1)
#else
#define VC_LIFT_DETECT      (0)
#endif

#define VC_STATE_MS         (1000u)               // STATE frame every 1 s.
#define VC_SPEED_MAX        (63u)                 // Largest speed field value.

static volatile vc_state_t g_vc_state = VC_STATE_IDLE;  // One writer: VC.
static uint32_t g_vc_speed    = 0u;             // Last speed sent, cm/s.
static uint32_t g_vc_obstacle = VC_OBS_CLEAR;   // Last obstacle level read.
static bool     g_vc_lifted   = false;          // Last lift level read.
static uint32_t g_vc_want_cmd = S2_MOTION_STOP; // Latest motion request.
static uint32_t g_vc_want_arg = 0u;
static uint32_t g_vc_sent_cmd = VC_MOTION_NONE; // Last one subsystem 2 took.
static uint32_t g_vc_sent_arg = VC_MOTION_NONE;

static void       vc_task(INT stacd, void * exinf);
static void       vc_poll_remote(void);
static void       vc_dispatch(vc_event_t ev);
static void       vc_enter_following(void);
static vc_event_t vc_obstacle_event(uint32_t level);
static void       vc_motion(uint32_t cmd, uint32_t arg);
static void       vc_motion_send(void);
static void       vc_set_state(vc_state_t next);
static uint32_t   vc_obstacle_level(void);
static bool       vc_is_lifted(void);
static char const * vc_state_name(vc_state_t state);
static void       vc_publish_state(void);

ER
vehicle_init (void)
{
    // No kernel objects of its own yet: the VC uses the other subsystems'.
    return (E_OK);
}

ER
vehicle_start (void)
{
    // Core 1 (TP_PRC2), with motion control; the radio service is on core 0.
    ID tid = app_start_task(vc_task, VC_PRI, TP_PRC2, "vc_task");

    return ((tid < E_OK) ? tid : E_OK);
}

vc_state_t
vc_get_state (void)
{
    return (g_vc_state);
}

static void
vc_task (INT stacd, void * exinf)
{
    (void) stacd;
    (void) exinf;

    SYSTIM   otm;
    uint32_t last_pub = 0u;

    vc_motion(S2_MOTION_STOP, 0u);              // Motors off at start.
    tm_printf((UB*)"vc: started, state %s, lift detection %s\n",
              vc_state_name(g_vc_state), VC_LIFT_DETECT ? "on" : "off");

    for (;;)
    {
        // Lifted or overturned: stop, and keep stopping while it lasts.
        // Read first, so a START in the same tick is already refused.
        g_vc_lifted = vc_is_lifted();
        if (g_vc_lifted)
        {
            vc_dispatch(VC_EV_LIFTED);
        }

        vc_poll_remote();       // Remote safety commands and START/RESET.

        g_vc_obstacle = vc_obstacle_level();
        vc_dispatch(vc_obstacle_event(g_vc_obstacle));

        vc_motion_send();       // Retries a request that did not fit.

        // Telemetry: the dashboard sees the state at least once a second.
        // Timed by the clock: a 10 ms delay can take up to 20 ms here.
        (void) tk_get_otm(&otm);
        if ((otm.lo - last_pub) >= VC_STATE_MS)
        {
            last_pub = otm.lo;
            vc_publish_state();
        }

        (void) tk_dly_tsk(VC_TICK_MS);
    }
}

static void
vc_poll_remote (void)
{
    UINT ptn = 0u;
    ER   err = tk_wai_flg(g_flg_remote, VC_REMOTE_EVENTS,
                          TWF_ORW | TWF_BITCLR, &ptn, TMO_POL);

    if (E_OK == err)
    {
        if (0u != (ptn & S1_FLG_ESTOP))
        {
            vc_dispatch(VC_EV_ESTOP);
        }
        else if (0u != (ptn & S1_FLG_STOP))
        {
            vc_dispatch(VC_EV_STOP);
        }
        else
        {
            // Only CMD_READY: handled below.
        }

        if (0u != (ptn & S1_FLG_CMD_READY))
        {
            uint8_t packed = 0u;
            INT     size   = 0;

            // One CMD_READY may stand for several commands: drain them all.
            do
            {
                size = tk_rcv_mbf(g_mbf_remote_cmd, &packed, TMO_POL);
                if (size > 0)
                {
                    uint32_t cmd = s1_cmd_of(packed);

                    // STOP and ESTOP already arrived through their flags.
                    if (S1_CMD_START == cmd)
                    {
                        vc_dispatch(VC_EV_START);
                    }
                    else if (S1_CMD_RESET == cmd)
                    {
                        vc_dispatch(VC_EV_RESET);
                    }
                    else
                    {
                        // STOP, ESTOP (flags) and MODE (not used yet).
                    }
                }
            } while (size > 0);
        }
    }

    return;
}

static void
vc_dispatch (vc_event_t ev)
{
    // Safety events are accepted in every state.
    if (VC_EV_ESTOP == ev)
    {
        vc_motion(S2_MOTION_STOP, 0u);
        vc_set_state(VC_STATE_FAULT);
        return;
    }

    if (VC_EV_STOP == ev)
    {
        vc_motion(S2_MOTION_STOP, 0u);
        if (VC_STATE_FAULT != g_vc_state)
        {
            vc_set_state(VC_STATE_STOPPED);
        }
        return;
    }

    if (VC_EV_LIFTED == ev)
    {
        vc_motion(S2_MOTION_STOP, 0u);
        if ((VC_STATE_IDLE != g_vc_state) && (VC_STATE_STOPPED != g_vc_state)
            && (VC_STATE_FAULT != g_vc_state))
        {
            vc_set_state(VC_STATE_STOPPED);
        }
        return;
    }

    // The wheels must not start while the car is in the air.
    if ((VC_EV_START == ev) && g_vc_lifted)
    {
        tm_printf((UB*)"vc: START ignored, car lifted\n");
        return;
    }

    // Every other event: only the current state's case runs.
    switch (g_vc_state)
    {
      case VC_STATE_IDLE:
        if (VC_EV_START == ev)
        {
            vc_enter_following();
        }
      break;

      case VC_STATE_FOLLOWING:
        if (VC_EV_OBS_EMERGENCY == ev)
        {
            vc_motion(S2_MOTION_STOP, 0u);
            vc_set_state(VC_STATE_BLOCKED);
        }
        else if (VC_EV_OBS_SLOW == ev)
        {
            vc_motion(S2_MOTION_SET_SPEED, VC_SLOW_CM_S);
        }
        else if (VC_EV_OBS_CLEAR == ev)
        {
            vc_motion(S2_MOTION_SET_SPEED, VC_CRUISE_CM_S);
        }
        else
        {
            // START and RESET are ignored while driving.
        }
      break;

      case VC_STATE_BLOCKED:
        if ((VC_EV_OBS_SLOW == ev) || (VC_EV_OBS_CLEAR == ev))
        {
            vc_set_state(VC_STATE_FOLLOWING);
            vc_motion(S2_MOTION_SET_SPEED, VC_SLOW_CM_S);
        }
      break;

      case VC_STATE_STOPPED:
        if (VC_EV_START == ev)
        {
            vc_enter_following();
        }
        else if (VC_EV_RESET == ev)
        {
            vc_motion(S2_MOTION_STOP, 0u);
            vc_set_state(VC_STATE_IDLE);
        }
        else
        {
            // Obstacle levels do not change a stopped car.
        }
      break;

      case VC_STATE_FAULT:
        if (VC_EV_RESET == ev)
        {
            vc_motion(S2_MOTION_STOP, 0u);
            vc_set_state(VC_STATE_IDLE);
        }
      break;

      case VC_STATE_TURNING:        // Reserved for stage 2.
      case VC_STATE_BYPASSING:      // Reserved for stage 3.
      default:
      break;
    }

    return;
}

static void
vc_enter_following (void)
{
    vc_set_state(VC_STATE_FOLLOWING);

    // Set the speed (or stop) for the obstacle level right now.
    g_vc_obstacle = vc_obstacle_level();
    vc_dispatch(vc_obstacle_event(g_vc_obstacle));

    return;
}

static vc_event_t
vc_obstacle_event (uint32_t level)
{
    vc_event_t ev = VC_EV_OBS_CLEAR;

    if (VC_OBS_EMERGENCY == level)
    {
        ev = VC_EV_OBS_EMERGENCY;
    }
    else if (VC_OBS_SLOW == level)
    {
        ev = VC_EV_OBS_SLOW;
    }
    else
    {
        // Clear.
    }

    return (ev);
}

static uint32_t
vc_obstacle_level (void)
{
    T_RFLG   rflg;
    uint32_t level = VC_OBS_CLEAR;

    // The S5 bits describe a level, so read them without clearing.
    if (E_OK == tk_ref_flg(flg_s5_obstacle, &rflg))
    {
        if (0u != (rflg.flgptn & S5_FLAG_EMERGENCY))
        {
            level = VC_OBS_EMERGENCY;
        }
        else if (0u != (rflg.flgptn & S5_FLAG_SLOWDOWN))
        {
            level = VC_OBS_SLOW;
        }
        else
        {
            level = VC_OBS_CLEAR;
        }
    }

    return (level);
}

static bool
vc_is_lifted (void)
{
    bool lifted = false;

#if VC_LIFT_DETECT
    T_RFLG rflg;

    // A level kept by subsystem 4: read it, never clear it.
    if ((E_OK == tk_ref_flg(flg_s4_imu, &rflg))
        && (0u != (rflg.flgptn & S4_FLAG_LIFTED)))
    {
        lifted = true;
    }
#endif

    return (lifted);
}

static void
vc_motion (uint32_t cmd, uint32_t arg)
{
    // Only the latest request matters: a newer one replaces an unsent one.
    g_vc_want_cmd = cmd;
    g_vc_want_arg = (arg > S2_MOTION_ARGUMENT_MAX) ? S2_MOTION_ARGUMENT_MAX
                                                   : arg;

    if (S2_MOTION_SET_SPEED == cmd)
    {
        g_vc_speed = g_vc_want_arg;
    }
    else if (S2_MOTION_STOP == cmd)
    {
        g_vc_speed = 0u;
    }
    else
    {
        // Distance and turns keep the last speed.
    }

    vc_motion_send();

    return;
}

static void
vc_motion_send (void)
{
    static bool s_busy = false;     // A send failed and is being retried.

    // Send only changes, so polling every tick does not flood subsystem 2.
    if ((g_vc_want_cmd != g_vc_sent_cmd) || (g_vc_want_arg != g_vc_sent_arg))
    {
        uint16_t req = motion_packRequest((uint8_t) g_vc_want_cmd,
                                          (uint16_t) g_vc_want_arg);
        ER       err = tk_snd_mbf(mbf_motion_req, &req, S2_MOTION_REQUEST_BYTES,
                                  TMO_POL);

        if (E_OK == err)
        {
            s_busy = false;
            g_vc_sent_cmd = g_vc_want_cmd;
            g_vc_sent_arg = g_vc_want_arg;
            tm_printf((UB*)"vc: motion cmd=%u arg=%u\n",
                      (unsigned int) g_vc_want_cmd,
                      (unsigned int) g_vc_want_arg);
        }
        else
        {
            // Buffer full: kept in g_vc_want_*, sent again next tick.
            // Printed once, not every tick.
            if (!s_busy)
            {
                s_busy = true;
                tm_printf((UB*)"vc: motion cmd=%u busy (%d), retrying\n",
                          (unsigned int) g_vc_want_cmd, (int) err);
            }
        }
    }

    return;
}

static void
vc_set_state (vc_state_t next)
{
    if (next != g_vc_state)
    {
        tm_printf((UB*)"vc: state %s -> %s\n", vc_state_name(g_vc_state),
                  vc_state_name(next));
        g_vc_state = next;
        vc_publish_state();                     // Show changes at once.
    }

    return;
}

static void
vc_publish_state (void)
{
    uint8_t      frame[PROTO_MAX_FRAME_BYTES];
    proto_bits_t bits;
    T_RFLG       rflg;
    uint32_t     link_lost = 0u;

    // LINK_LOST is a level kept by comms_task: read it, never clear it.
    if ((E_OK == tk_ref_flg(g_flg_remote, &rflg))
        && (0u != (rflg.flgptn & S1_FLG_LINK_LOST)))
    {
        link_lost = 1u;
    }

    proto_init(&bits, frame, sizeof(frame));
    proto_begin(&bits, PROTO_TYPE_STATE);
    proto_put(&bits, (uint32_t) g_vc_state, PROTO_ST_STATE_BITS);
    proto_put(&bits, g_vc_obstacle, PROTO_ST_OBSTACLE_BITS);
    proto_put(&bits, link_lost, PROTO_ST_LINK_LOST_BITS);
    proto_put_sat(&bits, (g_vc_speed > VC_SPEED_MAX) ? VC_SPEED_MAX
                                                     : g_vc_speed,
                  PROTO_ST_SPEED_BITS);

    // Never blocks; a full queue drops the frame and Comms counts it.
    (void) s1_publish(frame, proto_end(&bits), false);

    return;
}

static char const *
vc_state_name (vc_state_t state)
{
    static char const * const names[] = {"IDLE", "FOLLOWING", "TURNING",
        "BYPASSING", "BLOCKED", "STOPPED", "FAULT", "?"};

    return ((state < (sizeof(names) / sizeof(names[0]))) ? names[state] : "?");
}

/*** end of file ***/
