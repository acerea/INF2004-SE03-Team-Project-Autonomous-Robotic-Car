/** @file vehicle.c
 *
 * @brief Vehicle Controller (VC): mission state machine and obstacle bypass.
 *
 * @par
 * The VC polls its inputs every VC_TICK_MS,
 * because a task can wait on only one kernel object at a time, turns them
 * into events and handles each event in vc_dispatch() (switch on state).
 *
 * Change of requirement (week 12): subsystem 4 no longer reports
 * collisions. It reports LIFTED (robot picked up: the signal for the end
 * of the demonstration run) and OVERTURNED in flg_s4_imu (IF-13). The VC
 * reads them every tick, before anything else, and stops the motors.
 *
 * State table (ESTOP, STOP, LIFTED, OVERTURNED accepted in every state):
 *   any        ESTOP              -> FAULT      motors stop
 *   any        OVERTURNED         -> FAULT      motors stop (RESET to IDLE)
 *   not FAULT  STOP               -> STOPPED    motors stop
 *   not FAULT  LIFTED             -> STOPPED    motors stop: end of run
 *                                               (IDLE stays IDLE)
 *   IDLE       START              -> FOLLOWING  speed by obstacle level
 *   FOLLOWING  OBS_EMERGENCY      -> BYPASSING  S5 side selected, S2 sequence
 *   FOLLOWING  OBS_SLOW / CLEAR   -> FOLLOWING  speed 10 / 20 cm/s
 *   BLOCKED    OBS_EMERGENCY      -> BYPASSING  once S5 side flags are ready
 *   BLOCKED    OBS_SLOW / CLEAR   -> FOLLOWING  speed 10 cm/s
 *   STOPPED    START              -> FOLLOWING  speed by obstacle level
 *   STOPPED    RESET              -> IDLE       motors stop
 *   FAULT      RESET              -> IDLE       motors stop
 *   Any other state/event pair is ignored (e.g. START in FAULT).
 *   Guard: START is ignored while the robot is lifted or overturned.
 */
#include <stdint.h>
#include <stdbool.h>
#include "vehicle.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/ir_sense.h"
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
#define VC_EV_LIFTED        (7u)    // S4: robot picked up (rising edge).
#define VC_EV_OVERTURNED    (8u)    // S4: robot overturned (rising edge).
#define VC_EV_MOTION_DONE   (9u)    // S2 completed a distance/turn command.
#define VC_EV_REJOIN        (10u)   // S3 saw the guide line stably.

#define VC_HANDLED_BITS     (S4_FLG_LIFTED | S4_FLG_OVERTURNED)

// Keep-alive: laptop_station.py sends PING every 0.5 s. With no command at
// all for this long (station closed, Wi-Fi or broker gone) the car stops.
#define VC_LINK_TIMEOUT_MS  (2000u)

#define VC_STATE_MS         (1000u)
#define VC_BARCODE_TURN_ANGLE  (90u)
#define VC_BARCODE_UTURN_ANGLE (180u)
#define VC_BARCODE_STRAIGHT_CM (20u)               // STATE frame every 1 s.
#define VC_SPEED_MAX        (63u)                 // Largest speed field value.

static volatile vc_state_t g_vc_state = VC_STATE_IDLE;  // One writer: VC.
static uint32_t g_vc_speed    = 0u;             // Last speed sent, cm/s.
static uint32_t g_vc_obstacle = VC_OBS_CLEAR;   // Last obstacle level read.
static uint32_t g_vc_want_cmd = S2_MOTION_STOP; // Latest motion request.
static uint32_t g_vc_want_arg = 0u;
static uint32_t g_vc_sent_cmd = VC_MOTION_NONE; // Last one subsystem 2 took.
static uint32_t g_vc_sent_arg = VC_MOTION_NONE;
static UINT     g_vc_imu_bits = 0u;             // Last flg_s4_imu read.
static uint32_t g_vc_last_remote_ms = 0u;       // Last command (any) time.

typedef enum {
    VC_BYPASS_TURN_OUT = 0,
    VC_BYPASS_LATERAL_OUT,
    VC_BYPASS_FORWARD,
    VC_BYPASS_TURN_IN,
    VC_BYPASS_LATERAL_IN,
    VC_BYPASS_TURN_ALIGN,
    VC_BYPASS_WAIT_REJOIN
} vc_bypass_step_t;

static vc_bypass_step_t g_vc_bypass_step;
static S5_BYPASS_DIR    g_vc_bypass_dir;
static uint32_t         g_vc_bypass_started;
static uint8_t          g_vc_rejoin_samples;
static uint32_t         g_vc_bypass_lateral_cm = VC_BYPASS_LATERAL_CM;
static uint32_t         g_vc_bypass_forward_cm = VC_BYPASS_FORWARD_CM;
static bool              g_vc_rejoin_armed;

static void       vc_task(INT stacd, void * exinf);
static void       vc_poll_remote(void);
static void       vc_poll_navigation(void);
static void       vc_poll_line_following(void);
static void       vc_poll_motion_done(void);
static void       vc_poll_imu(void);
static void       vc_check_link(void);
static void       vc_poll_bypass(void);
static void       vc_dispatch(vc_event_t ev);
static void       vc_enter_following(void);
static vc_event_t vc_obstacle_event(uint32_t level);
static void       vc_motion(uint32_t cmd, uint32_t arg);
static void       vc_motion_send(void);
static void       vc_set_state(vc_state_t next);
static uint32_t   vc_obstacle_level(void);
static char const * vc_state_name(vc_state_t state);
static void       vc_publish_state(void);
static void       vc_start_bypass(void);
static void       vc_bypass_command(uint32_t cmd, uint32_t arg);
static bool       vc_rejoin_seen(void);
static void       vc_finish_bypass(bool rejoined);

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
    tm_printf((UB*)"vc: started, state %s\n", vc_state_name(g_vc_state));

    for (;;)
    {
        vc_poll_imu();          // Lifted / overturned: stop at once.
        vc_poll_remote();       // Safety commands next.
        vc_check_link();        // No laptop keep-alive: stop.
        vc_poll_motion_done();
        vc_poll_navigation();
        if (VC_STATE_FOLLOWING == g_vc_state)
        {
            vc_poll_line_following();
        }

        g_vc_obstacle = vc_obstacle_level();
        vc_dispatch(vc_obstacle_event(g_vc_obstacle));
        vc_poll_bypass();

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
                    SYSTIM   rx_tm;

                    // Any command, PING included, proves the laptop is there.
                    (void) tk_get_otm(&rx_tm);
                    g_vc_last_remote_ms = rx_tm.lo;

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
vc_poll_imu (void)
{
    T_RFLG rflg;
    UINT   bits;
    UINT   rising;

    // Level bits owned by subsystem 4: read, never clear.
    if (E_OK != tk_ref_flg(flg_s4_imu, &rflg))
    {
        return;
    }

    bits   = rflg.flgptn & S4_FLG_ALL;
    rising = bits & ~g_vc_imu_bits;

    if (0u != (rising & S4_FLG_IMU_FAULT))
    {
        // Lift detection is not available: report it, keep driving.
        tm_printf((UB*)"vc: WARNING IMU fault, lift stop unavailable\n");
    }

    g_vc_imu_bits = bits;
    ir_set_lift_override((0u != (bits & VC_HANDLED_BITS)));

    if (0u != (rising & S4_FLG_OVERTURNED))
    {
        vc_dispatch(VC_EV_OVERTURNED);
    }
    else if (0u != (rising & S4_FLG_LIFTED))
    {
        vc_dispatch(VC_EV_LIFTED);
    }
    else if ((0u != (bits & VC_HANDLED_BITS)) && (0u != g_vc_speed))
    {
        // Still held or overturned and somehow moving: stop again.
        vc_motion(S2_MOTION_STOP, 0u);
    }
    else
    {
        // Nothing new.
    }

    return;
}

static void
vc_check_link (void)
{
    SYSTIM   now;
    T_RFLG   rflg;
    uint32_t quiet_ms;
    bool     b_lost;
    bool     b_moving = (VC_STATE_FOLLOWING == g_vc_state)
                        || (VC_STATE_TURNING == g_vc_state)
                        || (VC_STATE_BYPASSING == g_vc_state)
                        || (VC_STATE_BLOCKED == g_vc_state);

    if (!b_moving)
    {
        return;
    }

    (void) tk_get_otm(&now);
    quiet_ms = now.lo - g_vc_last_remote_ms;
    b_lost   = (quiet_ms > VC_LINK_TIMEOUT_MS);

    // LINK_LOST is a level kept by comms_task: read it, never clear it.
    if ((E_OK == tk_ref_flg(g_flg_remote, &rflg))
        && (0u != (rflg.flgptn & S1_FLG_LINK_LOST)))
    {
        b_lost = true;
    }

    if (b_lost)
    {
        tm_printf((UB*)"vc: laptop link lost (quiet %u ms), stopping\n",
                  (unsigned int) quiet_ms);
        ir_lock_steering(false);
        s2_setLineCorrection(0);
        vc_dispatch(VC_EV_STOP);
    }

    return;
}

static void
vc_poll_navigation (void)
{
    ir_nav_event_t nav;

    while (ir_receive_nav_event(&nav))
    {
        if (VC_STATE_FOLLOWING != g_vc_state)
        {
            continue;
        }

        if (IR_NAV_LEFT == nav.cmd)
        {
            s2_setLineCorrection(0);
            ir_lock_steering(true);
            vc_set_state(VC_STATE_TURNING);
            vc_motion(S2_MOTION_TURN_LEFT, VC_BARCODE_TURN_ANGLE);
        }
        else if (IR_NAV_RIGHT == nav.cmd)
        {
            s2_setLineCorrection(0);
            ir_lock_steering(true);
            vc_set_state(VC_STATE_TURNING);
            vc_motion(S2_MOTION_TURN_RIGHT, VC_BARCODE_TURN_ANGLE);
        }
        else if (IR_NAV_STRAIGHT == nav.cmd)
        {
            s2_setLineCorrection(0);
            ir_lock_steering(true);
            vc_set_state(VC_STATE_TURNING);
            vc_motion(S2_MOTION_MOVE_DISTANCE, VC_BARCODE_STRAIGHT_CM);
        }
        else if (IR_NAV_UTURN == nav.cmd)
        {
            s2_setLineCorrection(0);
            ir_lock_steering(true);
            vc_set_state(VC_STATE_TURNING);
            vc_motion(S2_MOTION_TURN_LEFT, VC_BARCODE_UTURN_ANGLE);
        }
    }

    return;
}

static void
vc_poll_line_following (void)
{
    ir_snapshot_t snap;

    ir_get_telemetry(&snap);
    if ((IR_TRACK == snap.state)
        || ((IR_LINE_LOST == snap.state) && (0u == snap.lost_timeout)))
    {
        s2_setLineCorrection(snap.line_pos);
    }
    else if (IR_LINE_LOST == snap.state)
    {
        s2_setLineCorrection(0);
        vc_motion(S2_MOTION_STOP, 0u);
    }
    else
    {
        s2_setLineCorrection(0);
    }

    return;
}

static void
vc_poll_motion_done (void)
{
    UINT ptn = 0u;

    if ((VC_STATE_TURNING == g_vc_state)
        && (E_OK == tk_wai_flg(flg_motion, S2_MOTION_DONE,
                               TWF_ORW | TWF_BITCLR, &ptn, TMO_POL)))
    {
        if (0u != (ptn & S2_MOTION_DONE))
        {
            ir_lock_steering(false);
            s2_setLineCorrection(0);
            vc_enter_following();
        }
    }

    return;
}
static void
vc_dispatch (vc_event_t ev)
{
    // Safety events are accepted in every state.
    if ((VC_EV_ESTOP == ev) || (VC_EV_OVERTURNED == ev))
    {
        vc_motion(S2_MOTION_STOP, 0u);
        if (VC_EV_OVERTURNED == ev)
        {
            tm_printf((UB*)"vc: robot overturned, motors stopped\n");
        }
        ir_set_lift_override(true);
        vc_set_state(VC_STATE_FAULT);
        return;
    }

    if (VC_EV_LIFTED == ev)
    {
        // Lifting the robot is the end-of-run signal.
        vc_motion(S2_MOTION_STOP, 0u);
        tm_printf((UB*)"vc: robot lifted, end of run, motors stopped\n");
        if ((VC_STATE_FAULT != g_vc_state) && (VC_STATE_IDLE != g_vc_state))
        {
            ir_set_lift_override(true);
            vc_set_state(VC_STATE_STOPPED);
        }
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
            vc_start_bypass();
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
        if (VC_EV_OBS_EMERGENCY == ev)
        {
            /* S5 may publish EMERGENCY before its side sweep has set the
             * direction bits.  Retry on the next VC tick. */
            vc_start_bypass();
        }
        else if ((VC_EV_OBS_SLOW == ev) || (VC_EV_OBS_CLEAR == ev))
        {
            vc_set_state(VC_STATE_FOLLOWING);
            vc_motion(S2_MOTION_SET_SPEED, VC_SLOW_CM_S);
        }
      break;

      case VC_STATE_BYPASSING:
        if (VC_EV_MOTION_DONE == ev)
        {
            switch (g_vc_bypass_step)
            {
              case VC_BYPASS_TURN_OUT:
                g_vc_bypass_step = VC_BYPASS_LATERAL_OUT;
                vc_bypass_command(S2_MOTION_MOVE_DISTANCE,
                                  g_vc_bypass_lateral_cm);
              break;
              case VC_BYPASS_LATERAL_OUT:
                g_vc_bypass_step = VC_BYPASS_FORWARD;
                vc_bypass_command((g_vc_bypass_dir == S5_DIR_LEFT)
                                      ? S2_MOTION_TURN_RIGHT
                                      : S2_MOTION_TURN_LEFT,
                                  VC_BYPASS_TURN_DEG);
              break;
              case VC_BYPASS_FORWARD:
                g_vc_bypass_step = VC_BYPASS_TURN_IN;
                vc_bypass_command(S2_MOTION_MOVE_DISTANCE,
                                  g_vc_bypass_forward_cm);
              break;
              case VC_BYPASS_TURN_IN:
                /* Turn in towards the line: opposite of the first turn. */
                g_vc_bypass_step = VC_BYPASS_LATERAL_IN;
                vc_bypass_command((g_vc_bypass_dir == S5_DIR_LEFT)
                                      ? S2_MOTION_TURN_RIGHT
                                      : S2_MOTION_TURN_LEFT,
                                  VC_BYPASS_TURN_DEG);
              break;
              case VC_BYPASS_LATERAL_IN:
                g_vc_bypass_step = VC_BYPASS_TURN_ALIGN;
                vc_bypass_command(S2_MOTION_MOVE_DISTANCE,
                                  g_vc_bypass_lateral_cm);
              break;
              case VC_BYPASS_TURN_ALIGN:
                /* Line up with the original heading: same as the first turn. */
                g_vc_bypass_step = VC_BYPASS_WAIT_REJOIN;
                vc_bypass_command((g_vc_bypass_dir == S5_DIR_LEFT)
                                      ? S2_MOTION_TURN_LEFT
                                      : S2_MOTION_TURN_RIGHT,
                                  VC_BYPASS_TURN_DEG);
              break;
              default:
                /* Completion of the final turn only starts the explicit
                 * line rejoin window; it does not assume success. */
                {
                    SYSTIM now;
                    (void)tk_get_otm(&now);
                    g_vc_bypass_started = now.lo;
                    g_vc_rejoin_armed = true;
                }
              break;
            }
        }
        else if (VC_EV_REJOIN == ev)
        {
            vc_finish_bypass(true);
        }
      break;

      case VC_STATE_STOPPED:
        if (VC_EV_START == ev)
        {
            vc_enter_following();
        }
        else if (VC_EV_RESET == ev)
        {
            ir_set_lift_override(false);
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
            ir_set_lift_override(false);
            vc_motion(S2_MOTION_STOP, 0u);
            vc_set_state(VC_STATE_IDLE);
        }
      break;

      case VC_STATE_TURNING:        // Reserved for stage 2.
      default:
      break;
    }

    return;
}

    static void
    vc_start_bypass (void)
    {
        T_RFLG flags;
        UINT side = 0u;

        if (E_OK == tk_ref_flg(flg_s5_obstacle, &flags))
        {
            side = flags.flgptn & (S5_FLAG_AVOID_LEFT | S5_FLAG_AVOID_RIGHT);
        }
        if (S5_FLAG_AVOID_LEFT == side)
        {
            g_vc_bypass_dir = S5_DIR_LEFT;
        }
        else if (S5_FLAG_AVOID_RIGHT == side)
        {
            g_vc_bypass_dir = S5_DIR_RIGHT;
        }
        else
        {
            /* No sweep result is safer as a stop than choosing a side. */
            vc_motion(S2_MOTION_STOP, 0u);
            vc_set_state(VC_STATE_BLOCKED);
            return;
        }

        /* Sideways distance from S5's width estimate: half the obstacle,
         * half the car and a margin. Fixed distance if S5 has no width. */
        {
            uint32_t w = g_s5_width_cm;

            g_vc_bypass_lateral_cm = (0u == w) ? VC_BYPASS_LATERAL_CM
                : (w / 2u) + VC_CAR_HALF_WIDTH_CM + VC_BYPASS_MARGIN_CM;
            if (g_vc_bypass_lateral_cm < VC_BYPASS_LATERAL_MIN_CM)
            {
                g_vc_bypass_lateral_cm = VC_BYPASS_LATERAL_MIN_CM;
            }
            if (g_vc_bypass_lateral_cm > VC_BYPASS_LATERAL_MAX_CM)
            {
                g_vc_bypass_lateral_cm = VC_BYPASS_LATERAL_MAX_CM;
            }
            tm_printf((UB*)"vc: bypass %s, obstacle width %u cm, sideways %u cm\n",
                      (S5_DIR_LEFT == g_vc_bypass_dir) ? "left" : "right",
                      (unsigned int) w, (unsigned int) g_vc_bypass_lateral_cm);
        }

        /* Forward leg: the gap S5 measured when the car stopped, plus the
         * obstacle depth, half the car and a margin, so the car is fully
         * past the obstacle before it turns back in. */
        {
            uint32_t gap = g_s5_front_cm;

            g_vc_bypass_forward_cm = (0u == gap) ? VC_BYPASS_FORWARD_CM
                : gap + VC_OBSTACLE_DEPTH_CM + VC_CAR_HALF_LENGTH_CM
                  + VC_BYPASS_MARGIN_CM;
            if (g_vc_bypass_forward_cm < VC_BYPASS_FORWARD_MIN_CM)
            {
                g_vc_bypass_forward_cm = VC_BYPASS_FORWARD_MIN_CM;
            }
            if (g_vc_bypass_forward_cm > VC_BYPASS_FORWARD_MAX_CM)
            {
                g_vc_bypass_forward_cm = VC_BYPASS_FORWARD_MAX_CM;
            }
            tm_printf((UB*)"vc: bypass gap %u cm, forward %u cm\n",
                      (unsigned int) gap, (unsigned int) g_vc_bypass_forward_cm);
        }

        (void)tk_wai_flg(flg_motion, S2_MOTION_DONE, TWF_ORW | TWF_BITCLR,
                         &side, TMO_POL);
        g_vc_bypass_step = VC_BYPASS_TURN_OUT;
        g_vc_rejoin_samples = 0u;
        g_vc_rejoin_armed = false;
        g_vc_bypass_started = 0u;
        ir_lock_steering(true);
        vc_set_state(VC_STATE_BYPASSING);
        vc_bypass_command((g_vc_bypass_dir == S5_DIR_LEFT)
                              ? S2_MOTION_TURN_LEFT : S2_MOTION_TURN_RIGHT,
                          VC_BYPASS_TURN_DEG);
    }

    static void
    vc_bypass_command (uint32_t cmd, uint32_t arg)
    {
        SYSTIM now;

        vc_motion(cmd, arg);
        (void)tk_get_otm(&now);
        g_vc_bypass_started = now.lo;
    }

    static bool
    vc_rejoin_seen (void)
    {
        ir_snapshot_t snap;
        int pos;

        ir_get_telemetry(&snap);
        pos = snap.line_pos;
        if ((snap.state == IR_TRACK) && (snap.lost_timeout == 0u)
            && ((snap.sensors & IR_SENS_CENTRE) != 0u)
            && (pos >= -250) && (pos <= 250))
        {
            if (g_vc_rejoin_samples < VC_REJOIN_STABLE_SAMPLES)
            {
                g_vc_rejoin_samples++;
            }
        }
        else
        {
            g_vc_rejoin_samples = 0u;
        }
        return (g_vc_rejoin_samples >= VC_REJOIN_STABLE_SAMPLES);
    }

    static void
    vc_finish_bypass (bool rejoined)
    {
        ir_lock_steering(false);
        vc_motion(S2_MOTION_STOP, 0u);
        g_vc_rejoin_samples = 0u;
        g_vc_rejoin_armed = false;
        if (rejoined)
        {
            vc_set_state(VC_STATE_FOLLOWING);
            vc_motion(S2_MOTION_SET_SPEED, VC_CRUISE_CM_S);
        }
        else
        {
            vc_set_state(VC_STATE_BLOCKED);
        }
    }

    static void
    vc_poll_bypass (void)
    {
        SYSTIM now;
        UINT ptn = 0u;

        if (VC_STATE_BYPASSING != g_vc_state)
        {
            return;
        }
        if (E_OK == tk_wai_flg(flg_motion, S2_MOTION_DONE,
                               TWF_ORW | TWF_BITCLR, &ptn, TMO_POL))
        {
            vc_dispatch(VC_EV_MOTION_DONE);
        }
        if ((VC_BYPASS_WAIT_REJOIN == g_vc_bypass_step)
            && g_vc_rejoin_armed)
        {
            if (vc_rejoin_seen())
            {
                vc_dispatch(VC_EV_REJOIN);
            }
            else
            {
                (void)tk_get_otm(&now);
                if ((now.lo - g_vc_bypass_started) >= VC_REJOIN_TIMEOUT_MS)
                {
                    vc_finish_bypass(false);
                }
            }
        }
        else
        {
            (void)tk_get_otm(&now);
            if ((now.lo - g_vc_bypass_started) >= VC_BYPASS_TIMEOUT_MS)
            {
                vc_finish_bypass(false);
            }
        }
    }

static void
vc_enter_following (void)
{
    // Guard: never start the motors while lifted or overturned.
    if (0u != (g_vc_imu_bits & VC_HANDLED_BITS))
    {
        tm_printf((UB*)"vc: START ignored, robot lifted or overturned\n");
        return;
    }

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
