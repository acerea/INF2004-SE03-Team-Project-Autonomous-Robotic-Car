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
 *   FOLLOWING  OBS_SLOW / CLEAR   -> FOLLOWING  slow (5) / cruise speed
 *                                               (cruise: default 3 cm/s,
 *                                               set by remote MODE 1..15)
 *   TURNING    OBS_EMERGENCY      -> FOLLOWING  only during a barcode
 *                                               STRAIGHT move: stop, then
 *                                               the normal obstacle flow
 *   BLOCKED    OBS_EMERGENCY      -> BYPASSING  once S5 side flags are ready
 *   BLOCKED    OBS_SLOW / CLEAR   -> FOLLOWING  slow speed
 *   STOPPED    START              -> FOLLOWING  speed by obstacle level
 *   STOPPED    RESET              -> IDLE       motors stop
 *   FAULT      RESET              -> IDLE       motors stop
 *   Any other state/event pair is ignored (e.g. START in FAULT).
 *   Guard: START is ignored while the robot is lifted or overturned.
 *   Line lost for longer than the IR timeout holds the car stopped in
 *   FOLLOWING (g_vc_line_hold) until the line is seen again.
 *   Remote MODE: arg 0 = recalibrate the IR sensors (only when not
 *   driving), arg 1..15 = cruise speed in cm/s.
 */
#include <stdint.h>
#include <stdbool.h>
#include "vehicle.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem2/wheel_encoder.h"
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

#define VC_HANDLED_BITS     (S4_FLG_LIFTED | S4_FLG_OVERTURNED)

// EVENT frames (type 9): the console reasons, also sent over MQTT.
#define VC_EVT_LINK_LOST        (1u)    // arg: ms without a command.
#define VC_EVT_LINE_SEARCH      (2u)    // arg: 1 searching right, 0 left.
#define VC_EVT_LINE_FOUND       (3u)
#define VC_EVT_LINE_NOT_FOUND   (4u)
#define VC_EVT_OVERTURNED       (5u)
#define VC_EVT_LIFTED           (6u)
#define VC_EVT_OBST_BARCODE     (7u)
#define VC_EVT_BYPASS           (8u)    // arg: side (1 L, 2 R) << 8 | sideways cm.
#define VC_EVT_BYPASS_FWD       (9u)    // arg: forward leg cm.
#define VC_EVT_START_IGNORED    (10u)   // lifted or overturned.
#define VC_EVT_MOTION_BUSY      (11u)   // arg: command.
#define VC_EVT_CAL_SWEEP        (12u)
#define VC_EVT_CAL_TIMEOUT      (13u)   // arg: step.
#define VC_EVT_CAL_REFUSED      (14u)   // arg: 0 driving, 1 lifted/overturned.
#define VC_EVT_CAL_STATIC       (15u)   // FAULT: standing calibration.
#define VC_EVT_IMU_FAULT        (16u)
#define VC_EVT_CAL_CENTRED      (17u)   // Calibration done, middle on the line.
#define VC_EVT_CAL_NOT_CENTRED  (18u)   // Calibration done, line not found.
#define VC_EVT_BARCODE_PENDING  (19u)   // arg: letter << 8 | IR_NAV_xxx.
#define VC_EVT_JUNCTION         (20u)   // arg: IR_NAV_xxx being done.
#define VC_EVT_MOVE             (21u)   // arg: reverse << 8 | cm.
#define VC_EVT_MOVE_REFUSED     (22u)   // Nudge only while IDLE / STOPPED.
#define VC_EVT_BYPASS_LINE      (23u)   // Way back saw the line; arg: cm driven.
#define VC_EVT_BYPASS_NO_LINE   (24u)   // Return leg: no line; arg: cm driven.
#define VC_EVT_BYPASS_DONE      (25u)   // arg: 1 middle sensor on the line, 0 not.

// Keep-alive: laptop_station.py sends PING every 0.5 s. With no command at
// all for this long (station closed, Wi-Fi or broker gone) the car stops.
#define VC_LINK_TIMEOUT_MS  (2000u)

#define VC_STATE_MS         (1000u)
#define VC_BARCODE_TURN_ANGLE  (90u)
#define VC_BARCODE_LEFT_ANGLE  (100u)  /* Barcode LEFT came out short at 90 (26 Nov). */
/* U-turn asks for more than 180: encoder turns come out short and the
 * shortfall grows with the angle.
 * Trial and error: still short -> raise, past the line -> lower. */
#define VC_BARCODE_UTURN_ANGLE (210u)
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
static uint32_t g_vc_cruise_cm_s = VC_CRUISE_CM_S; // Set by remote MODE.
static bool     g_vc_line_hold = false;         // Stopped: line lost too long.
static uint8_t  g_vc_search_step = 0u;          // Line search: 0 off, 1 running.
static uint32_t g_vc_lost_since = 0u;           // When the VC lost the line (0 = sees it).

// Small-step turning ("nudges"), used by the line search and by the
// centring after calibration: turn a few degrees, stop, let the car settle,
// look, repeat. A fast continuous turn slid past the tape before it could
// stop. After the target is seen, it is checked again once the car has
// settled; if it overshot, it nudges back the other way.
typedef struct
{
    bool     b_right;       // Current direction.
    uint8_t  phase;         // 0 start nudge, 1 turning, 2 settling, 3 verifying.
    uint8_t  steps;         // Nudges done on this leg.
    uint8_t  max_steps;     // Nudges allowed on this leg.
    uint8_t  first_steps;   // Nudges allowed on the first leg.
    uint8_t  leg;           // 0 first side, 1 other side, 2 overshoot fix.
    uint8_t  fixes;         // Overshoot corrections done.
    uint32_t t_ms;
} vc_nudge_t;

static vc_nudge_t g_vc_search_nudge;
static vc_nudge_t g_vc_cal_nudge;
static bool       g_vc_cal_active = false;      // Sweep calibration running.
static uint8_t    g_vc_cal_step = 0u;           // 0 left, 1 right, 2 back.
static vc_state_t g_vc_cal_return = VC_STATE_IDLE;
static uint32_t   g_vc_cal_step_ms = 0u;

/* The leg the car is driving NOW (each MOTION_DONE starts the next one).
 * Out and past by measured distances; back in by the line sensors. */
typedef enum {
    VC_BYPASS_TURN_OUT = 0,     // Turn 90 towards the chosen side.
    VC_BYPASS_SIDE_OUT,         // Sideways: S5's clearance + half car + margin.
    VC_BYPASS_TURN_PARALLEL,    // Turn back to the original heading.
    VC_BYPASS_PASS,             // Drive past the obstacle.
    VC_BYPASS_TURN_IN,          // Turn 90 towards the line.
    VC_BYPASS_SEEK,             // Drive in until a line sensor sees black.
    VC_BYPASS_ADVANCE,          // Wheels onto the line (sensors are ahead).
    VC_BYPASS_ALIGN             // Pivot until the middle sensor is on it.
} vc_bypass_step_t;

static vc_bypass_step_t g_vc_bypass_step;
static S5_BYPASS_DIR    g_vc_bypass_dir;
static uint32_t         g_vc_bypass_started;
static uint8_t          g_vc_see_samples;       // Line seen this many ticks.
static bool             g_vc_align_left_line;   // ALIGN: middle saw white first.
static float            g_vc_seek_start_cm;     // Wheel distance at the start of SEEK.
static uint32_t         g_vc_bypass_lateral_cm = VC_BYPASS_LATERAL_CM;
static uint32_t         g_vc_bypass_forward_cm = VC_BYPASS_FORWARD_CM;

/*
 * Barcode letter -> action, set from the dashboard (S1_CMD_MAP) because the
 * letter/action pairs are only announced just before the demo.
 * 26 letters x 3 bits (minimum-bit change requirement), 10 per word:
 *   0-4 = IR_NAV_xxx (NONE, LEFT, RIGHT, STRAIGHT, UTURN)
 *   7   = VC_MAP_DEFAULT: not set, use subsystem 3's own command
 *         (A=LEFT, B=RIGHT, C=STRAIGHT, D=UTURN)
 * Only vc_task reads or writes it, so no locking is needed.
 */
#define VC_MAP_BITS         (3u)
#define VC_MAP_PER_WORD     (10u)
#define VC_MAP_LETTERS      (26u)
#define VC_MAP_DEFAULT      (PROTO_MAP_DEFAULT)
#define VC_MAP_ALL_DEFAULT  (0x3FFFFFFFu)   // 10 x 0b111.

static uint32_t g_vc_nav_map[3] =
{
    VC_MAP_ALL_DEFAULT, VC_MAP_ALL_DEFAULT, VC_MAP_ALL_DEFAULT
};

/* Barcode action waiting for the next T-junction (IR_NAV_NONE = none), and
 * the turn still to do after driving onto the junction (0 = none). */
static ir_nav_cmd_t g_vc_pending_cmd = IR_NAV_NONE;
static uint8_t      g_vc_pending_letter = 0u;
static uint32_t     g_vc_pending_ms = 0u;
static ir_nav_cmd_t g_vc_junction_turn = IR_NAV_NONE;

static void       vc_junction_start(void);

/* S2 MOVE arguments carry cm in 7 bits (s2_move_arg). */
#if (VC_BYPASS_FORWARD_MAX_CM > 127u) || (VC_BYPASS_LATERAL_MAX_CM > 127u) \
    || (VC_JUNCTION_ADVANCE_CM > 127u)
#error "S2 move distances must be <= 127 cm"
#endif
static void       vc_map_set(uint32_t letter, uint32_t action);
static uint32_t   vc_map_get(uint32_t letter);
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
static void       vc_finish_bypass(bool rejoined);
static void       vc_bypass_next(void);
static void       vc_bypass_cut(void);
static uint32_t   vc_bypass_turn(bool toward);
static void       vc_follow_speed(uint32_t speed);
static uint32_t   vc_slow_speed(void);
static void       vc_handle_mode(uint32_t arg);
static void       vc_event(uint32_t code, uint32_t arg);
static void       vc_poll_calibration(void);
static void       vc_cal_finish(bool b_centred);
static void       vc_check_ir(void);
static void       vc_nudge_start(vc_nudge_t * p_n, bool b_right, uint32_t first_deg);
static uint8_t    vc_nudge_poll(vc_nudge_t * p_n, bool b_found);
static void       vc_start_calibration(void);
static bool       vc_take_motion_done(void);
static uint32_t   vc_now_ms(void);

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

        vc_poll_calibration();
        vc_check_ir();

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
            uint8_t msg[S1_CMD_MAX_MSG] = {0u};
            uint8_t packed = 0u;
            INT     size   = 0;

            // One CMD_READY may stand for several commands: drain them all.
            do
            {
                size = tk_rcv_mbf(g_mbf_remote_cmd, msg, TMO_POL);
                packed = msg[0];
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
                    else if (S1_CMD_MODE == cmd)
                    {
                        vc_handle_mode(s1_arg_of(packed));
                    }
                    else if ((S1_CMD_MAP == cmd) && (size >= 2))
                    {
                        vc_map_set(msg[1], s1_arg_of(packed));
                    }
                    else if ((S1_CMD_MOVE == cmd) && (size >= 2))
                    {
                        bool b_back = (0u != (s1_arg_of(packed)
                                              & PROTO_MOVE_REVERSE));

                        // Nudge only when the car is not on a run.
                        if ((VC_STATE_IDLE == g_vc_state)
                            || (VC_STATE_STOPPED == g_vc_state))
                        {
                            (void) vc_take_motion_done();
                            vc_motion(S2_MOTION_STOP, 0u);  // So a repeat
                                                            // is resent.
                            // At the cruise speed (dashboard), not
                            // subsystem 2's 10 cm/s stopped default.
                            vc_motion(b_back ? S2_MOTION_MOVE_BACK
                                             : S2_MOTION_MOVE_DISTANCE,
                                      s2_move_arg(msg[1],
                                                  g_vc_cruise_cm_s));
                            vc_event(VC_EVT_MOVE,
                                     ((b_back ? 1u : 0u) << 8) | msg[1]);
                            tm_printf((UB*)"vc: nudge %s %u cm at %u cm/s\n",
                                      b_back ? "back" : "forward",
                                      (unsigned int) msg[1],
                                      (unsigned int) g_vc_cruise_cm_s);
                        }
                        else
                        {
                            vc_event(VC_EVT_MOVE_REFUSED, 0u);
                            tm_printf((UB*)"vc: nudge refused, state %s\n",
                                      vc_state_name(g_vc_state));
                        }
                    }
                    else
                    {
                        // STOP, ESTOP (flags) and PING.
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
        vc_event(VC_EVT_IMU_FAULT, 0u);
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
                        || (VC_STATE_BLOCKED == g_vc_state)
                        || (VC_STATE_CALIBRATING == g_vc_state);

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
        vc_event(VC_EVT_LINK_LOST, quiet_ms);
        tm_printf((UB*)"vc: laptop link lost (quiet %u ms), stopping\n",
                  (unsigned int) quiet_ms);
        ir_lock_steering(false);
        s2_set_line_correction(0);
        vc_dispatch(VC_EV_STOP);
    }

    return;
}

static void
vc_map_set (uint32_t letter, uint32_t action)
{
    static char const * const names[8] =
    {
        "NONE", "LEFT", "RIGHT", "STRAIGHT", "UTURN", "?", "?", "DEFAULT"
    };
    uint32_t word;
    uint32_t shift;

    if ((letter >= VC_MAP_LETTERS)
        || ((action > IR_NAV_UTURN) && (VC_MAP_DEFAULT != action)))
    {
        return;
    }

    word  = letter / VC_MAP_PER_WORD;
    shift = (letter % VC_MAP_PER_WORD) * VC_MAP_BITS;
    g_vc_nav_map[word] = (g_vc_nav_map[word]
                          & ~(((1u << VC_MAP_BITS) - 1u) << shift))
                         | (action << shift);

    tm_printf((UB*)"vc: barcode map %c -> %s\n",
              (int) ('A' + letter), names[action]);

    return;
}

static uint32_t
vc_map_get (uint32_t letter)
{
    uint32_t word;
    uint32_t shift;

    if (letter >= VC_MAP_LETTERS)
    {
        return (VC_MAP_DEFAULT);
    }

    word  = letter / VC_MAP_PER_WORD;
    shift = (letter % VC_MAP_PER_WORD) * VC_MAP_BITS;

    return ((g_vc_nav_map[word] >> shift) & ((1u << VC_MAP_BITS) - 1u));
}

static void
vc_poll_navigation (void)
{
    ir_nav_event_t nav;

    while (ir_receive_nav_event(&nav))
    {
        uint32_t mapped;

        if (VC_STATE_FOLLOWING != g_vc_state)
        {
            continue;
        }

        // Dashboard mapping wins; DEFAULT keeps subsystem 3's command.
        mapped = vc_map_get(nav.letter);
        if (VC_MAP_DEFAULT != mapped)
        {
            nav.cmd = (ir_nav_cmd_t) mapped;
        }
        if (nav.letter < VC_MAP_LETTERS)
        {
            tm_printf((UB*)"vc: barcode %c -> cmd %u%s\n",
                      (int) ('A' + nav.letter), (unsigned int) nav.cmd,
                      (VC_MAP_DEFAULT != mapped) ? " (dashboard map)" : "");
        }

        if (IR_NAV_NONE == nav.cmd)
        {
            continue;               // Unknown letter or IGNORE: carry on.
        }

        // Do it at the next T-junction, not here: the barcode is beside
        // the line some way before the junction.
        g_vc_pending_cmd    = nav.cmd;
        g_vc_pending_letter = nav.letter;
        g_vc_pending_ms     = vc_now_ms();
        vc_event(VC_EVT_BARCODE_PENDING,
                 ((uint32_t) nav.letter << 8) | (uint32_t) nav.cmd);
        tm_printf((UB*)"vc: action %u waits for the junction\n",
                  (unsigned int) nav.cmd);
    }

    return;
}

/* T-junction reached with a barcode action waiting. STRAIGHT drives on
 * across; a turn first drives the wheels onto the junction (the sensors
 * are ahead of them), then turns there (vc_poll_motion_done). */
static void
vc_junction_start (void)
{
    ir_nav_cmd_t cmd = g_vc_pending_cmd;

    g_vc_pending_cmd = IR_NAV_NONE;
    vc_event(VC_EVT_JUNCTION, (uint32_t) cmd);
    tm_printf((UB*)"vc: junction, doing barcode %c action %u\n",
              (int) ('A' + g_vc_pending_letter), (unsigned int) cmd);

    s2_set_line_correction(0);
    ir_lock_steering(true);
    vc_set_state(VC_STATE_TURNING);
    (void) vc_take_motion_done();           // No stale "done".
    vc_motion(S2_MOTION_STOP, 0u);

    if (IR_NAV_STRAIGHT == cmd)
    {
        /* At cruise speed: after the STOP above S2's own speed is 0, and
         * it then used its 10 cm/s default (run 26 Nov 13:57: 11 cm/s). */
        vc_motion(S2_MOTION_MOVE_DISTANCE,
                  s2_move_arg(VC_BARCODE_STRAIGHT_CM, g_vc_cruise_cm_s));
    }
    else
    {
        g_vc_junction_turn = cmd;           // Set after vc_set_state.
        vc_motion(S2_MOTION_MOVE_DISTANCE,
                  s2_move_arg(VC_JUNCTION_ADVANCE_CM, g_vc_cruise_cm_s));
    }

    return;
}

static void
vc_poll_line_following (void)
{
    ir_snapshot_t snap;

    ir_get_telemetry(&snap);

    // A search is running: nudge until any sensor sees the line (and still
    // sees it once the car has settled).
    if (0u != g_vc_search_step)
    {
        uint8_t res = vc_nudge_poll(&g_vc_search_nudge, (0u != snap.sensors));

        s2_set_line_correction(0);
        if (1u == res)
        {
            vc_event(VC_EVT_LINE_FOUND, 0u);
            tm_printf((UB*)"vc: line found again\n");
            g_vc_search_step = 0u;
            g_vc_line_hold = false;
            vc_motion(S2_MOTION_STOP, 0u);  // The dispatch resends the speed.
            (void) vc_take_motion_done();
        }
        else if (2u == res)
        {
            vc_event(VC_EVT_LINE_NOT_FOUND, 0u);
            tm_printf((UB*)"vc: line not found, holding stopped\n");
            g_vc_search_step = 0u;
            g_vc_line_hold = true;
            vc_motion(S2_MOTION_STOP, 0u);
        }
        else
        {
            // Still searching.
        }
        return;
    }

    // A barcode action is waiting: S3 reports a junction (C + R on the
    // bar, run 26 Nov 14:01:16), or the bar is under the barcode (left)
    // sensor AND the right sensor together. Not left + centre: on a left
    // bend the line sits under those two, and that was taken for the
    // junction (13:57:07). Not straight after the barcode itself.
    if ((IR_NAV_NONE != g_vc_pending_cmd)
        && ((vc_now_ms() - g_vc_pending_ms) >= VC_JUNCTION_GUARD_MS)
        && (((IR_BARCODE == snap.state)
             && (0u != (snap.sensors & IR_SENS_RIGHT)))
            || (IR_JUNC_NONE != snap.junction)))
    {
        vc_junction_start();
        return;
    }

    if (IR_TRACK == snap.state)
    {
        // On the line: steer by it, release a hold. The obstacle dispatch
        // later in this tick sends the speed again.
        g_vc_lost_since = 0u;
        g_vc_line_hold = false;
        s2_set_line_correction(ir_line_pos(&snap));
    }
    else if (IR_BARCODE == snap.state)
    {
        // The barcode sensor (left) sees black: a barcode, a junction bar,
        // or the line itself sliding left on a LEFT bend. Not "lost": reset
        // the timer. Keep steering by C/R (still updated while reading):
        // going straight here ran the car off every left bend (video
        // 25 Nov 17:44). Line gone from C/R and last seen on the left:
        // it is under the barcode sensor, so curve left.
        g_vc_lost_since = 0u;
        g_vc_line_hold = false;
        if (0u != (snap.sensors & (IR_SENS_CENTRE | IR_SENS_RIGHT)))
        {
            s2_set_line_correction(ir_line_pos(&snap));
        }
        else if (ir_line_pos(&snap) <= -VC_LOST_ARC_MIN_POS)
        {
            s2_set_line_correction(-1000);        // Curve left.
        }
        else if (ir_line_pos(&snap) >= VC_LOST_ARC_MIN_POS)
        {
            s2_set_line_correction(1000);         // Curve right.
        }
        else
        {
            s2_set_line_correction(0);
        }
    }
    else if ((IR_LINE_LOST == snap.state)
             && ((0u == g_vc_lost_since)
                 || ((vc_now_ms() - g_vc_lost_since) < VC_LOST_STRAIGHT_MS)))
    {
        // Just lost it. Off a bend (last seen well to one side): curve
        // towards that side at full steering, because driving straight
        // took the car further from a line that curves away (video 25 Nov:
        // ran wide on the loop, then stop-and-search). Lost while nearly
        // centred (a gap): keep going straight. The timer is the VC's own,
        // started here while driving, so a "lost" left over from before
        // START does not start a search at once.
        if (0u == g_vc_lost_since)
        {
            g_vc_lost_since = vc_now_ms();
            if (0u == g_vc_lost_since)
            {
                g_vc_lost_since = 1u;
            }
        }
        if (ir_line_pos(&snap) >= VC_LOST_ARC_MIN_POS)
        {
            s2_set_line_correction(1000);         // Curve right.
        }
        else if (ir_line_pos(&snap) <= -VC_LOST_ARC_MIN_POS)
        {
            s2_set_line_correction(-1000);        // Curve left.
        }
        else
        {
            s2_set_line_correction(0);
        }
    }
    else if (IR_LINE_LOST == snap.state)
    {
        // Lost for longer than the IR timeout: search in small nudges,
        // first towards the side the line was last seen, then the other
        // way. Only if both fail, stop and HOLD (without the hold the
        // obstacle dispatch would restart the motors every tick).
        s2_set_line_correction(0);
        if (!g_vc_line_hold)
        {
            g_vc_lost_since  = 0u;
            g_vc_search_step = 1u;
            vc_event(VC_EVT_LINE_SEARCH, (ir_line_pos(&snap) >= 0) ? 1u : 0u);
            tm_printf((UB*)"vc: line lost, searching %s\n",
                      (ir_line_pos(&snap) >= 0) ? "right" : "left");
            vc_motion(S2_MOTION_STOP, 0u);
            vc_nudge_start(&g_vc_search_nudge, (ir_line_pos(&snap) >= 0),
                           VC_SEARCH_DEG);
        }
        else
        {
            vc_motion(S2_MOTION_STOP, 0u);
        }
    }
    else
    {
        s2_set_line_correction(0);
    }

    return;
}

static void
vc_poll_motion_done (void)
{
    UINT ptn = 0u;

    if ((VC_STATE_IDLE == g_vc_state) || (VC_STATE_STOPPED == g_vc_state))
    {
        (void) vc_take_motion_done();   // End of a nudge: nothing to do.
        return;
    }

    if ((VC_STATE_TURNING == g_vc_state)
        && (E_OK == tk_wai_flg(flg_motion, S2_MOTION_DONE,
                               TWF_ORW | TWF_BITCLR, &ptn, TMO_POL)))
    {
        if ((0u != (ptn & S2_MOTION_DONE))
            && (IR_NAV_NONE != g_vc_junction_turn))
        {
            // Wheels are now over the junction: do the barcode's turn.
            ir_nav_cmd_t turn = g_vc_junction_turn;

            g_vc_junction_turn = IR_NAV_NONE;
            vc_motion(S2_MOTION_STOP, 0u);
            if (IR_NAV_LEFT == turn)
            {
                vc_motion(S2_MOTION_TURN_LEFT, VC_BARCODE_LEFT_ANGLE);
            }
            else if (IR_NAV_RIGHT == turn)
            {
                vc_motion(S2_MOTION_TURN_RIGHT, VC_BARCODE_TURN_ANGLE);
            }
            else
            {
                vc_motion(S2_MOTION_TURN_LEFT, VC_BARCODE_UTURN_ANGLE);
            }
        }
        else if (0u != (ptn & S2_MOTION_DONE))
        {
            ir_lock_steering(false);
            s2_set_line_correction(0);
            vc_enter_following();
        }
        else
        {
            // No motion finished this tick.
        }
    }

    return;
}
static void
vc_dispatch (vc_event_t ev)
{
    // Safety events are accepted in every state.
    // Lift / overturn only matter while a run is active. At rest (IDLE,
    // STOPPED, FAULT) the motors are already off, and picking the robot up
    // to carry or rewire it used to throw it into FAULT. START stays
    // refused while either is set (vc_enter_following).
    if (((VC_EV_OVERTURNED == ev) || (VC_EV_LIFTED == ev))
        && ((VC_STATE_IDLE == g_vc_state) || (VC_STATE_STOPPED == g_vc_state)
            || (VC_STATE_FAULT == g_vc_state)))
    {
        vc_motion(S2_MOTION_STOP, 0u);  // Ends a nudge (else already off).
        return;
    }

    if ((VC_EV_ESTOP == ev) || (VC_EV_OVERTURNED == ev))
    {
        vc_motion(S2_MOTION_STOP, 0u);
        if (VC_EV_OVERTURNED == ev)
        {
            vc_event(VC_EVT_OVERTURNED, 0u);
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
        vc_event(VC_EVT_LIFTED, 0u);
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
            vc_follow_speed(vc_slow_speed());
        }
        else if (VC_EV_OBS_CLEAR == ev)
        {
            vc_follow_speed(g_vc_cruise_cm_s);
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
            vc_follow_speed(vc_slow_speed());
        }
        else
        {
            // Other events do not change BLOCKED.
        }
      break;

      case VC_STATE_BYPASSING:
        if (VC_EV_MOTION_DONE == ev)
        {
            vc_bypass_next();
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

      case VC_STATE_TURNING:
        // Turns are on the spot, but a barcode STRAIGHT is a 20 cm drive:
        // stop for an obstacle, then hand over to the FOLLOWING flow
        // (bypass once S5 has picked a side, BLOCKED until then).
        if ((VC_EV_OBS_EMERGENCY == ev)
            && (S2_MOTION_MOVE_DISTANCE == g_vc_want_cmd))
        {
            UINT ptn = 0u;

            vc_event(VC_EVT_OBST_BARCODE, 0u);
            tm_printf((UB*)"vc: obstacle during barcode move, stopping\n");
            vc_motion(S2_MOTION_STOP, 0u);
            (void) tk_wai_flg(flg_motion, S2_MOTION_DONE,
                              TWF_ORW | TWF_BITCLR, &ptn, TMO_POL);
            vc_enter_following();
        }
      break;

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

        /* Sideways: how far the obstacle reaches past the line on the
         * chosen side (S5's sweep), plus half the car and a margin. The
         * old "half the width" assumed the obstacle was centred on the
         * line. No sweep size at all: the fixed distance. */
        {
            uint32_t w = g_s5_width_cm;

            g_vc_bypass_lateral_cm = (0u == g_s5_front_cm) ? VC_BYPASS_LATERAL_CM
                : g_s5_clear_cm + VC_CAR_HALF_WIDTH_CM + VC_BYPASS_MARGIN_CM;
            if (g_vc_bypass_lateral_cm < VC_BYPASS_LATERAL_MIN_CM)
            {
                g_vc_bypass_lateral_cm = VC_BYPASS_LATERAL_MIN_CM;
            }
            if (g_vc_bypass_lateral_cm > VC_BYPASS_LATERAL_MAX_CM)
            {
                g_vc_bypass_lateral_cm = VC_BYPASS_LATERAL_MAX_CM;
            }
            vc_event(VC_EVT_BYPASS, (((S5_DIR_LEFT == g_vc_bypass_dir) ? 1u : 2u) << 8) | (g_vc_bypass_lateral_cm & 0xFFu));
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
            vc_event(VC_EVT_BYPASS_FWD, g_vc_bypass_forward_cm);
            tm_printf((UB*)"vc: bypass gap %u cm, forward %u cm\n",
                      (unsigned int) gap, (unsigned int) g_vc_bypass_forward_cm);
        }

        (void)tk_wai_flg(flg_motion, S2_MOTION_DONE, TWF_ORW | TWF_BITCLR,
                         &side, TMO_POL);
        g_vc_bypass_step = VC_BYPASS_TURN_OUT;
        g_vc_see_samples = 0u;
        g_vc_align_left_line = false;
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

    /* Distance driven forward (cm), both wheels averaged. */
    static float
    vc_wheel_cm (void)
    {
        return 0.5f * (encoder_get_left_distance() + encoder_get_right_distance());
    }

    /* Turn towards the chosen side (toward = true) or back (false). */
    static uint32_t
    vc_bypass_turn (bool toward)
    {
        bool left = (S5_DIR_LEFT == g_vc_bypass_dir);

        return (left == toward) ? S2_MOTION_TURN_LEFT : S2_MOTION_TURN_RIGHT;
    }

    /* The leg that was running finished: start the next one. */
    static void
    vc_bypass_next (void)
    {
        uint32_t seek_cm;

        switch (g_vc_bypass_step)
        {
          case VC_BYPASS_TURN_OUT:
            g_vc_bypass_step = VC_BYPASS_SIDE_OUT;
            vc_bypass_command(S2_MOTION_MOVE_DISTANCE, g_vc_bypass_lateral_cm);
          break;
          case VC_BYPASS_SIDE_OUT:
            g_vc_bypass_step = VC_BYPASS_TURN_PARALLEL;
            vc_bypass_command(vc_bypass_turn(false), VC_BYPASS_TURN_DEG);
          break;
          case VC_BYPASS_TURN_PARALLEL:
            g_vc_see_samples = 0u;
            g_vc_bypass_step = VC_BYPASS_PASS;
            vc_bypass_command(S2_MOTION_MOVE_DISTANCE, g_vc_bypass_forward_cm);
          break;
          case VC_BYPASS_PASS:
            g_vc_bypass_step = VC_BYPASS_TURN_IN;
            vc_bypass_command(vc_bypass_turn(false), VC_BYPASS_TURN_DEG);
          break;
          case VC_BYPASS_TURN_IN:
            /* Drive towards the line; vc_poll_bypass stops on it. */
            seek_cm = g_vc_bypass_lateral_cm + VC_BYPASS_SEEK_EXTRA_CM;
            if (seek_cm > 127u)
            {
                seek_cm = 127u;
            }
            g_vc_see_samples = 0u;
            g_vc_seek_start_cm = vc_wheel_cm();
            g_vc_bypass_step = VC_BYPASS_SEEK;
            vc_bypass_command(S2_MOTION_MOVE_DISTANCE, seek_cm);
          break;
          case VC_BYPASS_SEEK:
            /* Drove the whole way and never saw it. Turn back to the
             * original heading anyway; line following searches. */
            vc_event(VC_EVT_BYPASS_NO_LINE,
                     g_vc_bypass_lateral_cm + VC_BYPASS_SEEK_EXTRA_CM);
            tm_printf((UB*)"vc: bypass return, no line found\n");
            g_vc_align_left_line = false;
            g_vc_see_samples = 0u;
            g_vc_bypass_step = VC_BYPASS_ALIGN;
            vc_bypass_command(vc_bypass_turn(true), VC_BYPASS_TURN_DEG);
          break;
          case VC_BYPASS_ADVANCE:
            g_vc_align_left_line = false;
            g_vc_see_samples = 0u;
            g_vc_bypass_step = VC_BYPASS_ALIGN;
            vc_bypass_command(vc_bypass_turn(true), VC_BYPASS_ALIGN_MAX_DEG);
          break;
          case VC_BYPASS_ALIGN:
          default:
            /* Turned all the way without the middle sensor finding it. */
            vc_finish_bypass(false);
          break;
        }
    }

    /* Stop the leg that is running (line seen) without a stale "done". */
    static void
    vc_bypass_cut (void)
    {
        vc_motion(S2_MOTION_STOP, 0u);
        (void) vc_take_motion_done();
    }

    /* rejoined: the middle sensor found the line. Either way the car goes
     * back to FOLLOWING: line following steers onto it or searches, which
     * beats standing in BLOCKED off the line (log 18:26:31). */
    static void
    vc_finish_bypass (bool rejoined)
    {
        ir_lock_steering(false);
        vc_motion(S2_MOTION_STOP, 0u);
        g_vc_see_samples = 0u;
        vc_event(VC_EVT_BYPASS_DONE, rejoined ? 1u : 0u);
        tm_printf((UB*)"vc: bypass done, %s\n",
                  rejoined ? "on the line" : "line not under the middle sensor");
        g_vc_line_hold = false;
        g_vc_lost_since = 0u;
        vc_set_state(VC_STATE_FOLLOWING);
        vc_follow_speed(g_vc_cruise_cm_s);
    }

    static void
    vc_poll_bypass (void)
    {
        SYSTIM now;
        UINT ptn = 0u;
        ir_snapshot_t snap;

        if (VC_STATE_BYPASSING != g_vc_state)
        {
            return;
        }

        ir_get_telemetry(&snap);

        if (VC_BYPASS_SEEK == g_vc_bypass_step)
        {
            /* Any sensor on black, met at an angle (so the barcode sensor
             * may be first) -- but only near where the original line is.
             * Passing and turning in are NOT watched: the obstacle sits on
             * the original line, so a line there is another one (video
             * 25 Nov 18:55 followed the S-bend beside the box). */
            float gate_cm = (float)g_vc_bypass_lateral_cm
                            - (float)VC_JUNCTION_ADVANCE_CM
                            - (float)VC_BYPASS_SEEK_EARLY_CM;
            float driven_cm = vc_wheel_cm() - g_vc_seek_start_cm;

            if ((0u != snap.sensors) && (driven_cm >= gate_cm))
            {
                g_vc_see_samples++;
            }
            else
            {
                g_vc_see_samples = 0u;
            }
            if (g_vc_see_samples >= VC_BYPASS_SEE_SAMPLES)
            {
                vc_bypass_cut();
                vc_event(VC_EVT_BYPASS_LINE, (uint32_t)(driven_cm + 0.5f));
                tm_printf((UB*)"vc: bypass return, line seen after %d cm, onto it\n",
                          (int)(driven_cm + 0.5f));
                g_vc_see_samples = 0u;
                g_vc_bypass_step = VC_BYPASS_ADVANCE;
                vc_bypass_command(S2_MOTION_MOVE_DISTANCE,
                                  VC_JUNCTION_ADVANCE_CM);
                return;
            }
        }
        else if (VC_BYPASS_ALIGN == g_vc_bypass_step)
        {
            /* The sensors swing round over the line: wait for the C/R pair
             * to be off it first, then stop on the FIRST reading of either
             * C or R on it. The pivot sweeps them across the 18 mm line so
             * fast that the middle sensor alone saw it for one reading
             * (run R4, 26 Nov 13:21:46) and the turn ran its full 135 deg.
             * Line following centres the car from there. */
            if (0u == (snap.sensors & (IR_SENS_CENTRE | IR_SENS_RIGHT)))
            {
                g_vc_align_left_line = true;
            }
            else if (g_vc_align_left_line)
            {
                vc_bypass_cut();
                vc_finish_bypass(true);
                return;
            }
            else
            {
                /* Still on the bit it crossed: not yet. */
            }
        }
        else
        {
            /* Out and past: measured legs, no sensors. */
        }

        if (E_OK == tk_wai_flg(flg_motion, S2_MOTION_DONE,
                               TWF_ORW | TWF_BITCLR, &ptn, TMO_POL))
        {
            vc_dispatch(VC_EV_MOTION_DONE);
            return;
        }

        (void)tk_get_otm(&now);
        if ((now.lo - g_vc_bypass_started) >= VC_BYPASS_TIMEOUT_MS)
        {
            tm_printf((UB*)"vc: bypass leg %u timed out\n",
                      (unsigned int) g_vc_bypass_step);
            vc_finish_bypass(false);
        }
    }

static void
vc_enter_following (void)
{
    // Guard: never start the motors while lifted or overturned.
    if (0u != (g_vc_imu_bits & VC_HANDLED_BITS))
    {
        vc_event(VC_EVT_START_IGNORED, 0u);
        tm_printf((UB*)"vc: START ignored, robot lifted or overturned\n");
        return;
    }

    g_vc_line_hold = false;
    g_vc_search_step = 0u;
    g_vc_lost_since = 0u;
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
        uint16_t req = motion_pack_request((uint8_t) g_vc_want_cmd,
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
                vc_event(VC_EVT_MOTION_BUSY, g_vc_want_cmd);
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
    // The IR steering lock belongs to TURNING and BYPASSING only. Leaving
    // them any other way (STOP, ESTOP, lift, overturn, link loss) used to
    // leave it on, so after the next START no barcode was ever read and
    // the line-lost timeout never fired.
    if ((VC_STATE_TURNING != next) && (VC_STATE_BYPASSING != next))
    {
        ir_lock_steering(false);
    }

    // A line search only exists while FOLLOWING.
    if (VC_STATE_FOLLOWING != next)
    {
        g_vc_search_step = 0u;
    }

    // A junction turn only exists while TURNING; a waiting barcode action
    // only while a run is going on.
    if (VC_STATE_TURNING != next)
    {
        g_vc_junction_turn = IR_NAV_NONE;
    }
    if ((VC_STATE_IDLE == next) || (VC_STATE_STOPPED == next)
        || (VC_STATE_FAULT == next))
    {
        g_vc_pending_cmd = IR_NAV_NONE;
    }

    // Leaving CALIBRATING any way but the normal end (STOP, ESTOP, lift,
    // link loss, timeout): keep the old IR levels.
    if ((VC_STATE_CALIBRATING != next) && g_vc_cal_active)
    {
        g_vc_cal_active = false;
        ir_cal_sweep_cancel();
    }

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
    proto_put_sat(&bits, g_vc_cruise_cm_s, PROTO_ST_CRUISE_BITS);

    // Never blocks; a full queue drops the frame and Comms counts it.
    (void) s1_publish(frame, proto_end(&bits), false);

    return;
}

static uint32_t
vc_now_ms (void)
{
    SYSTIM now;

    (void) tk_get_otm(&now);
    return (now.lo);
}

// Consume S2_MOTION_DONE if it is set (never waits).
static bool
vc_take_motion_done (void)
{
    UINT ptn = 0u;

    return (E_OK == tk_wai_flg(flg_motion, S2_MOTION_DONE,
                               TWF_ORW | TWF_BITCLR, &ptn, TMO_POL));
}

// Sweep calibration: turn left, right, back to the start heading on the
// spot, while the IR task records each sensor's darkest and lightest value.
static void
vc_start_calibration (void)
{
    vc_event(VC_EVT_CAL_SWEEP, 0u);
    tm_printf((UB*)"vc: sweep calibration (keep the robot over the line)\n");
    g_vc_cal_return = g_vc_state;
    g_vc_cal_step   = 0u;
    (void) vc_take_motion_done();
    ir_cal_sweep_begin();
    vc_set_state(VC_STATE_CALIBRATING);
    g_vc_cal_active  = true;
    g_vc_cal_step_ms = vc_now_ms();
    vc_motion(S2_MOTION_TURN_LEFT, VC_CAL_SWEEP_DEG);

    return;
}

static void
vc_poll_calibration (void)
{
    ir_snapshot_t snap;
    uint32_t      now;

    if (VC_STATE_CALIBRATING != g_vc_state)
    {
        return;
    }

    now = vc_now_ms();

    // Steps 0-2: the sweep (left, right, back to the start heading).
    if (g_vc_cal_step <= 2u)
    {
        if (vc_take_motion_done())
        {
            g_vc_cal_step++;
            g_vc_cal_step_ms = now;
            if (1u == g_vc_cal_step)
            {
                vc_motion(S2_MOTION_TURN_RIGHT, 2u * VC_CAL_SWEEP_DEG);
            }
            else if (2u == g_vc_cal_step)
            {
                vc_motion(S2_MOTION_TURN_LEFT, VC_CAL_SWEEP_DEG);
            }
            else
            {
                // Sweep finished: keep the new levels, then centre.
                g_vc_cal_active = false;
                ir_cal_sweep_end();
                vc_motion(S2_MOTION_STOP, 0u);
            }
        }
        else if ((now - g_vc_cal_step_ms) >= VC_CAL_STEP_TIMEOUT_MS)
        {
            vc_event(VC_EVT_CAL_TIMEOUT, g_vc_cal_step);
            tm_printf((UB*)"vc: calibration turn timed out, old levels kept\n");
            vc_motion(S2_MOTION_STOP, 0u);
            vc_set_state(g_vc_cal_return);      // Cancels the sweep.
        }
        else
        {
            // Calibration turn still running.
        }
        return;
    }

    // Steps 3-4: centre the middle sensor on the line, using the new levels.
    ir_get_telemetry(&snap);

    // A frozen IR task leaves an old snapshot: never trust it.
    if ((now - snap.timestamp_ms) > 500u)
    {
        tm_printf((UB*)"vc: IR task not responding, cannot check the line\n");
        vc_motion(S2_MOTION_STOP, 0u);
        vc_cal_finish(false);
        return;
    }

    if (3u == g_vc_cal_step)
    {
        // Wait for the IR task to apply the levels (it leaves CALI).
        if (IR_CALI != snap.state)
        {
            if (0u != (snap.sensors & IR_SENS_CENTRE))
            {
                vc_cal_finish(true);
            }
            else
            {
                // Towards the side the line is on (+ = right); with no
                // sensor on the line, the side it was last seen.
                bool b_right = (ir_line_pos(&snap) >= 0);

                tm_printf((UB*)"vc: centring on the line, nudging %s\n",
                          b_right ? "right" : "left");
                g_vc_cal_step    = 4u;
                g_vc_cal_step_ms = now;
                vc_nudge_start(&g_vc_cal_nudge, b_right, VC_CENTRE_DEG);
            }
        }
        else if ((now - g_vc_cal_step_ms) >= VC_CENTRE_WAIT_MS)
        {
            tm_printf((UB*)"vc: IR did not leave calibration, not centring\n");
            vc_cal_finish(false);
        }
        else
        {
            // Still waiting for the IR task.
        }
        return;
    }

    // Step 4: nudge until the middle sensor is (and stays) on the line.
    {
        uint8_t res = vc_nudge_poll(&g_vc_cal_nudge,
                                    (0u != (snap.sensors & IR_SENS_CENTRE)));

        if (1u == res)
        {
            vc_cal_finish(true);
        }
        else if ((2u == res)
                 || ((now - g_vc_cal_step_ms) >= VC_CENTRE_MAX_MS))
        {
            vc_cal_finish(false);
        }
        else
        {
            // Still nudging.
        }
    }

    return;
}

// DEBUG: the IR task updates its snapshot every 2-10 ms. Say so on the
// console when it stops (and when it comes back), once each time.
static void
vc_check_ir (void)
{
    static bool b_warned = false;
    ir_snapshot_t snap;
    uint32_t      quiet;

    ir_get_telemetry(&snap);
    if (0u == snap.timestamp_ms)
    {
        return;                         // IR task not started yet.
    }
    quiet = vc_now_ms() - snap.timestamp_ms;

    if ((quiet > 1000u) && !b_warned)
    {
        b_warned = true;
        tm_printf((UB*)"vc: WARNING IR task not responding (last update %u ms ago), "
                  "IR stage %u ch %u\n", (unsigned int) quiet,
                  (unsigned int) (ir_debug_stage() & 0xFFu),
                  (unsigned int) (ir_debug_stage() >> 8));
    }
    else if ((quiet <= 1000u) && b_warned)
    {
        b_warned = false;
        tm_printf((UB*)"vc: IR task responding again\n");
    }
    else
    {
        // No change in IR task health.
    }

    return;
}

static void
vc_nudge_start (vc_nudge_t * p_n, bool b_right, uint32_t first_deg)
{
    uint32_t n = first_deg / VC_NUDGE_DEG;

    p_n->b_right     = b_right;
    p_n->phase       = 0u;
    p_n->steps       = 0u;
    p_n->first_steps = (uint8_t) ((n < 1u) ? 1u : ((n > 60u) ? 60u : n));
    p_n->max_steps   = p_n->first_steps;
    p_n->leg         = 0u;
    p_n->fixes       = 0u;
    p_n->t_ms        = vc_now_ms();

    return;
}

// One tick of a nudge search. b_found: the target is seen now.
// Returns 0 still going, 1 found (and still there after settling), 2 gave up.
static uint8_t
vc_nudge_poll (vc_nudge_t * p_n, bool b_found)
{
    uint32_t now = vc_now_ms();

    switch (p_n->phase)
    {
      case 0u:      // Start one nudge. STOP first, so an identical turn
                    // request is still sent (vc_motion only sends changes).
        (void) vc_take_motion_done();
        vc_motion(S2_MOTION_STOP, 0u);
        vc_motion(p_n->b_right ? S2_MOTION_TURN_RIGHT : S2_MOTION_TURN_LEFT,
                  VC_NUDGE_DEG);
        p_n->phase = 1u;
        p_n->t_ms  = now;
      break;

      case 1u:      // Turning.
        if (b_found)
        {
            vc_motion(S2_MOTION_STOP, 0u);
            (void) vc_take_motion_done();
            p_n->phase = 3u;
            p_n->t_ms  = now;
        }
        else if (vc_take_motion_done()
                 || ((now - p_n->t_ms) >= VC_NUDGE_TURN_MAX_MS))
        {
            vc_motion(S2_MOTION_STOP, 0u);
            p_n->phase = 2u;
            p_n->t_ms  = now;
        }
        else
        {
            // Still turning.
        }
      break;

      case 2u:      // Stopped between nudges: let it settle, then look.
        if (b_found)
        {
            p_n->phase = 3u;
            p_n->t_ms  = now;
        }
        else if ((now - p_n->t_ms) >= VC_NUDGE_SETTLE_MS)
        {
            p_n->steps++;
            if (p_n->steps < p_n->max_steps)
            {
                p_n->phase = 0u;
            }
            else if (0u == p_n->leg)
            {
                // Not that way: the other way, back past the start.
                p_n->leg       = 1u;
                p_n->b_right   = !p_n->b_right;
                p_n->steps     = 0u;
                p_n->max_steps = (uint8_t) (2u * p_n->first_steps);
                p_n->phase     = 0u;
            }
            else
            {
                return (2u);
            }
        }
        else
        {
            // Settling.
        }
      break;

      default:      // 3: seen it and stopped; check again once settled.
        if ((now - p_n->t_ms) >= VC_NUDGE_SETTLE_MS)
        {
            if (b_found)
            {
                return (1u);
            }
            if (p_n->fixes >= VC_NUDGE_FIXES_MAX)
            {
                return (2u);
            }
            // Slid past it: small nudges back the other way.
            p_n->fixes++;
            p_n->leg       = 2u;
            p_n->b_right   = !p_n->b_right;
            p_n->steps     = 0u;
            p_n->max_steps = VC_NUDGE_FIX_STEPS;
            p_n->phase     = 0u;
            tm_printf((UB*)"vc: overshot, nudging back %s\n",
                      p_n->b_right ? "right" : "left");
        }
      break;
    }

    return (0u);
}

// End of calibration (the new levels are already kept).
static void
vc_cal_finish (bool b_centred)
{
    if (b_centred)
    {
        vc_event(VC_EVT_CAL_CENTRED, 0u);
        tm_printf((UB*)"vc: calibration done, middle sensor on the line\n");
    }
    else
    {
        vc_event(VC_EVT_CAL_NOT_CENTRED, 0u);
        tm_printf((UB*)"vc: calibration done, line NOT found under the middle sensor\n");
    }
    vc_motion(S2_MOTION_STOP, 0u);
    vc_set_state(g_vc_cal_return);

    return;
}

// Speed while following: a line-lost hold wins over any obstacle speed.
static void
vc_follow_speed (uint32_t speed)
{
    if (0u != g_vc_search_step)
    {
        // A line-search turn is running: leave it alone.
    }
    else if (g_vc_line_hold)
    {
        vc_motion(S2_MOTION_STOP, 0u);
    }
    else
    {
        vc_motion(S2_MOTION_SET_SPEED, speed);
    }

    return;
}

// Obstacle slow-down speed, never faster than the chosen cruise speed.
static uint32_t
vc_slow_speed (void)
{
    return ((g_vc_cruise_cm_s < VC_SLOW_CM_S) ? g_vc_cruise_cm_s
                                              : VC_SLOW_CM_S);
}

// Remote MODE: 0 = recalibrate IR, 1..15 = cruise speed (cm/s).
static void
vc_handle_mode (uint32_t arg)
{
    if (0u == arg)
    {
        if ((VC_STATE_IDLE == g_vc_state) || (VC_STATE_STOPPED == g_vc_state))
        {
            if (0u != (g_vc_imu_bits & VC_HANDLED_BITS))
            {
                vc_event(VC_EVT_CAL_REFUSED, 1u);
                tm_printf((UB*)"vc: calibration refused, robot lifted or overturned\n");
            }
            else
            {
                vc_start_calibration();
            }
        }
        else if (VC_STATE_FAULT == g_vc_state)
        {
            // Motors must not move in FAULT: standing-still calibration.
            vc_event(VC_EVT_CAL_STATIC, 0u);
            tm_printf((UB*)"vc: IR recalibration requested (no sweep in FAULT)\n");
            ir_request_calibration();
        }
        else
        {
            vc_event(VC_EVT_CAL_REFUSED, 0u);
            tm_printf((UB*)"vc: recalibration refused while driving\n");
        }
    }
    else
    {
        uint32_t speed = (arg > S2_MOTION_MAX_SPEED) ? S2_MOTION_MAX_SPEED
                                                     : arg;

        g_vc_cruise_cm_s = speed;
        tm_printf((UB*)"vc: cruise speed %u cm/s\n", (unsigned int) speed);

        if (VC_STATE_FOLLOWING == g_vc_state)
        {
            vc_dispatch(vc_obstacle_event(vc_obstacle_level()));
        }
    }

    vc_publish_state();                     // Dashboard sees it at once.

    return;
}

// One EVENT frame. Never blocks; a full queue drops it (Comms counts it).
static void
vc_event (uint32_t code, uint32_t arg)
{
    uint8_t      frame[PROTO_MAX_FRAME_BYTES];
    proto_bits_t bits;

    proto_init(&bits, frame, sizeof(frame));
    proto_begin(&bits, PROTO_TYPE_EVENT);
    proto_put_sat(&bits, code, PROTO_EV_CODE_BITS);
    proto_put_sat(&bits, arg, PROTO_EV_ARG_BITS);
    (void) s1_publish(frame, proto_end(&bits), false);

    return;
}

static char const *
vc_state_name (vc_state_t state)
{
    static char const * const names[] = {"IDLE", "FOLLOWING", "TURNING",
        "BYPASSING", "BLOCKED", "STOPPED", "FAULT", "CALIBRATING"};

    return ((state < (sizeof(names) / sizeof(names[0]))) ? names[state] : "?");
}

/*** end of file ***/
