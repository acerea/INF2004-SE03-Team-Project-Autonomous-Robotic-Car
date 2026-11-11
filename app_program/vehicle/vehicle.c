/** @file vehicle.c
 *
 * @brief Vehicle Controller (VC): mission state machine, stage 1.
 *
 * @par
 * Stage 1: START / STOP / ESTOP / RESET from the laptop, and obstacle
 * slow-down and stop from subsystem 5. Turning (S3) and bypassing (S5)
 * are added in later stages. The VC polls its inputs every VC_TICK_MS,
 * because a task can wait on only one kernel object at a time.
 */
#include <stdint.h>
#include <stdbool.h>
#include "vehicle.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem5/subsystem5.h"

// 1 = motion requests are only printed. Set to 0 once subsystem 2 provides
// its request buffer (mbf_motion_req) and the send in vc_motion() is added.
#define VC_MOTION_STUB      (1)

#define VC_REMOTE_EVENTS    (S1_FLG_ESTOP | S1_FLG_STOP | S1_FLG_CMD_READY)

// Obstacle level from subsystem 5, worst first.
#define VC_OBS_CLEAR        (0u)
#define VC_OBS_SLOW         (1u)
#define VC_OBS_EMERGENCY    (2u)

#define VC_STATE_MS         (1000u)               // STATE frame every 1 s.
#define VC_SPEED_MAX        (63u)                 // Largest speed field value.

static volatile vc_state_t g_vc_state = VC_STATE_IDLE;  // One writer: VC.
static uint32_t g_vc_speed    = 0u;             // Last speed sent, cm/s.
static uint32_t g_vc_obstacle = VC_OBS_CLEAR;   // Last obstacle level read.

static void       vc_task(INT stacd, void * exinf);
static void       vc_motion(uint32_t cmd, uint32_t arg);
static void       vc_set_state(vc_state_t next);
static uint32_t   vc_obstacle_level(void);
static void       vc_handle_command(uint8_t packed);
static void       vc_handle_remote(void);
static void       vc_handle_obstacle(void);
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

    vc_motion(VC_MOTION_STOP, 0u);              // Motors off at start.
    tm_printf((UB*)"vc: started, state %s\n", vc_state_name(g_vc_state));

    for (;;)
    {
        vc_handle_remote();     // Safety commands first.
        vc_handle_obstacle();

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
vc_handle_remote (void)
{
    UINT ptn = 0u;
    ER   err = tk_wai_flg(g_flg_remote, VC_REMOTE_EVENTS,
                          TWF_ORW | TWF_BITCLR, &ptn, TMO_POL);

    if (E_OK == err)
    {
        if (0u != (ptn & S1_FLG_ESTOP))
        {
            vc_motion(VC_MOTION_STOP, 0u);
            vc_set_state(VC_STATE_FAULT);
        }
        else if (0u != (ptn & S1_FLG_STOP))
        {
            vc_motion(VC_MOTION_STOP, 0u);
            if (VC_STATE_FAULT != g_vc_state)
            {
                vc_set_state(VC_STATE_STOPPED);
            }
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
                    vc_handle_command(packed);
                }
            } while (size > 0);
        }
    }

    return;
}

static void
vc_handle_command (uint8_t packed)
{
    uint32_t cmd = s1_cmd_of(packed);

    switch (cmd)
    {
      case S1_CMD_START:
        if ((VC_STATE_IDLE == g_vc_state) || (VC_STATE_STOPPED == g_vc_state))
        {
            vc_set_state(VC_STATE_FOLLOWING);
            vc_handle_obstacle();               // Sets the speed.
        }
      break;

      case S1_CMD_RESET:
        if ((VC_STATE_FAULT == g_vc_state) || (VC_STATE_STOPPED == g_vc_state))
        {
            vc_motion(VC_MOTION_STOP, 0u);
            vc_set_state(VC_STATE_IDLE);
        }
      break;

      case S1_CMD_STOP:     // Already handled through the STOP flag.
      case S1_CMD_ESTOP:    // Already handled through the ESTOP flag.
      case S1_CMD_MODE:     // Not used yet.
      default:
      break;
    }

    return;
}

static void
vc_handle_obstacle (void)
{
    uint32_t level = vc_obstacle_level();

    g_vc_obstacle = level;

    if (VC_STATE_FOLLOWING == g_vc_state)
    {
        if (VC_OBS_EMERGENCY == level)
        {
            vc_motion(VC_MOTION_STOP, 0u);
            vc_set_state(VC_STATE_BLOCKED);
        }
        else
        {
            vc_motion(VC_MOTION_SPEED, (VC_OBS_SLOW == level)
                                       ? VC_SLOW_CM_S : VC_CRUISE_CM_S);
        }
    }
    else if (VC_STATE_BLOCKED == g_vc_state)
    {
        if (VC_OBS_EMERGENCY != level)
        {
            vc_set_state(VC_STATE_FOLLOWING);
            vc_motion(VC_MOTION_SPEED, VC_SLOW_CM_S);
        }
    }
    else
    {
        // Not driving: obstacles do not change the state.
    }

    return;
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
    static uint32_t s_last_cmd = 0xFFFFu;
    static uint32_t s_last_arg = 0xFFFFu;

    // Send only changes, so polling every tick does not flood subsystem 2.
    if ((cmd != s_last_cmd) || (arg != s_last_arg))
    {
        s_last_cmd = cmd;
        s_last_arg = arg;

        if (VC_MOTION_SPEED == cmd)
        {
            g_vc_speed = arg;
        }
        else if (VC_MOTION_STOP == cmd)
        {
            g_vc_speed = 0u;
        }
        else
        {
            // Distance and turns keep the last speed.
        }

#if VC_MOTION_STUB
        tm_printf((UB*)"vc: motion cmd=%u arg=%u (stub)\n",
                  (unsigned int) cmd, (unsigned int) arg);
#else
        // TODO: pack cmd (3 bits) and arg (13 bits) into 2 bytes and send to
        // subsystem 2's mbf_motion_req with TMO_POL once it exists.
#endif
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
