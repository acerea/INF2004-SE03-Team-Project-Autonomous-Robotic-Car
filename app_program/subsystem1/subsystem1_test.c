/** @file subsystem1_test.c
 *
 * @brief On-target tests for subsystem 1. Built only with: make TEST=1
 *
 * @par
 * Record results in testing/subsystem1/TEST_LOG.md. IDs: S1-U01 ...
 * NOTE: S1-U04 to S1-U06 share one sequence state, so run them in order.
 */
#include "subsystem1.h"
#include "../test_report.h"

#if APP_TEST

#if TM_MQTT
#include "mqtt_utk.h"
#endif

#define TEST_BURST      (60u)
#define TEST_MAX_MS     (20u)   // System tick is 10 ms, so allow two ticks.
#define TEST_SEQ_A      (10u)
#define TEST_SEQ_OLD    (9u)    // Delta 255 from TEST_SEQ_A.
#define TEST_SEQ_EDGE_X (138u)  // Delta 128: just outside the window.
#define TEST_SEQ_EDGE_I (137u)  // Delta 127: just inside the window.
#define TEST_BAD_CMD    (5u)    // Above S1_CMD_MODE, so unknown.
#define TEST_WIDE_U     (200u)  // Too wide for 7 bits.
#define TEST_WIDE_S     (-600)  // Too low for 10 signed bits.
#define TEST_U7_BITS    (7u)
#define TEST_U7_MAX     (127u)
#define TEST_S10_BITS   (10u)
#define TEST_S10_MIN    (-512)
#define TEST_S10_MAX    (511)
#define TEST_SEQ_BASE   (50u)   // Last accepted sequence for S1-U09.
#define TEST_SEQ_FRESH  (60u)   // Delta 10: would be accepted if length passed.
#define TEST_MARKER_LEN (1u)    // Size of the library's oversize marker.

static void
make_cmd (uint8_t * p_buf, uint32_t seq, uint32_t ver, uint32_t cmd,
          uint32_t arg)
{
    proto_bits_t bits;

    proto_init(&bits, p_buf, S1_RAW_MAX_MSG);
    proto_put(&bits, seq, PROTO_SEQ_BITS);
    proto_put(&bits, ver, PROTO_VERSION_BITS);
    proto_put(&bits, cmd, S1_CMD_BITS);
    proto_put(&bits, arg, S1_ARG_BITS);

    return;
}

EXPORT void
subsystem1_test (void)
{
    TEST_INIT();

    T_RMBF         rmbf;
    T_RFLG         rflg;
    uint8_t        raw[S1_RAW_MAX_MSG] = {0};
    uint8_t        packed = 0u;
    s1_seq_state_t seq_state = {0u, false};

    // S1-U01: every kernel object exists.
    TEST_CHECK("S1-U01",
               (E_OK == tk_ref_mbf(g_mbf_telemetry, &rmbf))
               && (E_OK == tk_ref_mbf(g_mbf_cmd_raw, &rmbf))
               && (E_OK == tk_ref_mbf(g_mbf_remote_cmd, &rmbf))
               && (E_OK == tk_ref_flg(g_flg_remote, &rflg))
               && (E_OK == tk_ref_flg(g_flg_comms, &rflg)),
               "kernel objects exist");

    // S1-U02: unsigned and signed fields round-trip at their limits.
    {
        uint8_t      frame[PROTO_MAX_FRAME_BYTES];
        proto_bits_t wr;
        proto_bits_t rd;

        proto_init(&wr, frame, sizeof(frame));
        proto_put(&wr, 0u, TEST_U7_BITS);
        proto_put(&wr, TEST_U7_MAX, TEST_U7_BITS);
        proto_put_s(&wr, TEST_S10_MIN, TEST_S10_BITS);
        proto_put_s(&wr, TEST_S10_MAX, TEST_S10_BITS);

        proto_open(&rd, frame, sizeof(frame));
        TEST_CHECK("S1-U02",
                   (false == wr.b_error)
                   && (0u == proto_get(&rd, TEST_U7_BITS))
                   && (TEST_U7_MAX == proto_get(&rd, TEST_U7_BITS))
                   && (TEST_S10_MIN == proto_get_s(&rd, TEST_S10_BITS))
                   && (TEST_S10_MAX == proto_get_s(&rd, TEST_S10_BITS)),
                   "pack/unpack round trip at min and max");
    }

    // S1-U03: out-of-range values saturate and raise the error flag.
    {
        uint8_t      frame[PROTO_MAX_FRAME_BYTES];
        proto_bits_t wr;
        proto_bits_t rd;

        proto_init(&wr, frame, sizeof(frame));
        proto_put_sat(&wr, TEST_WIDE_U, TEST_U7_BITS);
        proto_put_s(&wr, TEST_WIDE_S, TEST_S10_BITS);

        proto_open(&rd, frame, sizeof(frame));
        TEST_CHECK("S1-U03",
                   wr.b_error
                   && (TEST_U7_MAX == proto_get(&rd, TEST_U7_BITS))
                   && (TEST_S10_MIN == proto_get_s(&rd, TEST_S10_BITS)),
                   "out-of-range values saturate and flag an error");
    }

    // S1-U04: a valid command is accepted, a duplicate is rejected, and a
    // duplicate STOP is still accepted.
    make_cmd(raw, TEST_SEQ_A, PROTO_VERSION, S1_CMD_MODE, 9u);
    {
        bool b_first = s1_cmd_check(raw, S1_CMD_FRAME_BYTES, &seq_state,
                                    &packed);
        bool b_pack  = (S1_CMD_MODE == s1_cmd_of(packed))
                       && (9u == s1_arg_of(packed));
        bool b_dup   = s1_cmd_check(raw, S1_CMD_FRAME_BYTES, &seq_state,
                                    &packed);

        make_cmd(raw, TEST_SEQ_A, PROTO_VERSION, S1_CMD_STOP, 0u);
        TEST_CHECK("S1-U04",
                   b_first && b_pack && (false == b_dup)
                   && s1_cmd_check(raw, S1_CMD_FRAME_BYTES, &seq_state,
                                   &packed),
                   "valid accepted, duplicate rejected, STOP always ok");
    }

    // S1-U05: sequence window edges (uses the state left by S1-U04).
    {
        bool b_old;
        bool b_edge_out;
        bool b_edge_in;

        make_cmd(raw, TEST_SEQ_OLD, PROTO_VERSION, S1_CMD_START, 0u);
        b_old = s1_cmd_check(raw, S1_CMD_FRAME_BYTES, &seq_state, &packed);

        make_cmd(raw, TEST_SEQ_EDGE_X, PROTO_VERSION, S1_CMD_START, 0u);
        b_edge_out = s1_cmd_check(raw, S1_CMD_FRAME_BYTES, &seq_state,
                                  &packed);

        make_cmd(raw, TEST_SEQ_EDGE_I, PROTO_VERSION, S1_CMD_START, 0u);
        b_edge_in = s1_cmd_check(raw, S1_CMD_FRAME_BYTES, &seq_state,
                                 &packed);

        TEST_CHECK("S1-U05", (false == b_old) && (false == b_edge_out)
                   && b_edge_in, "sequence window edges");
    }

    // S1-U06: bad version, unknown command and bad length are rejected.
    {
        bool b_ver;
        bool b_cmd;
        bool b_short;
        bool b_long;

        seq_state.b_have_seq = false;   // Start fresh: only validity counts.

        make_cmd(raw, 1u, PROTO_VERSION + 1u, S1_CMD_START, 0u);
        b_ver = s1_cmd_check(raw, S1_CMD_FRAME_BYTES, &seq_state, &packed);

        make_cmd(raw, 2u, PROTO_VERSION, TEST_BAD_CMD, 0u);
        b_cmd = s1_cmd_check(raw, S1_CMD_FRAME_BYTES, &seq_state, &packed);

        make_cmd(raw, 3u, PROTO_VERSION, S1_CMD_START, 0u);
        seq_state.b_have_seq = false;
        b_short = s1_cmd_check(raw, S1_CMD_FRAME_BYTES - 1u, &seq_state,
                               &packed);
        seq_state.b_have_seq = false;
        b_long = s1_cmd_check(raw, S1_CMD_FRAME_BYTES + 1u, &seq_state,
                              &packed);

        TEST_CHECK("S1-U06",
                   (false == b_ver) && (false == b_cmd)
                   && (false == b_short) && (false == b_long),
                   "bad version, cmd or length rejected");
    }

    // S1-U07: publishing into a full queue returns and never blocks.
    {
        uint8_t  frame[PROTO_MAX_FRAME_BYTES] = {0};
        uint8_t  sink[S1_TLM_MAX_MSG];
        uint32_t drops = 0u;
        SYSTIM   t_start;
        SYSTIM   t_end;

        (void) tk_get_otm(&t_start);

        for (uint32_t idx = 0u; idx < TEST_BURST; idx++)
        {
            if (E_OK != s1_publish(frame, sizeof(frame), false))
            {
                drops++;
            }
        }

        (void) tk_get_otm(&t_end);

        TEST_CHECK("S1-U07",
                   (drops > 0u) && ((t_end.lo - t_start.lo) <= TEST_MAX_MS),
                   "full queue drops frames and never blocks");

        while (tk_rcv_mbf(g_mbf_telemetry, sink, TMO_POL) > 0)
        {
            // Empty the queue so later tests start clean.
        }

        (void) tk_clr_flg(g_flg_comms, 0u);     // Clear the drop flag.
    }

#if TM_MQTT
    // S1-U08: the frame type sits where the MQTT library looks for it.
    {
        uint8_t      frame[PROTO_MAX_FRAME_BYTES];
        proto_bits_t wr;
        uint32_t     type;

        proto_init(&wr, frame, sizeof(frame));
        proto_begin(&wr, PROTO_TYPE_OBSTACLE);
        type = (((uint32_t) frame[MQTT_UTK_TYPE_BYTE]
                 >> MQTT_UTK_TYPE_SHIFT) & MQTT_UTK_TYPE_MASK);

        TEST_CHECK("S1-U08", PROTO_TYPE_OBSTACLE == type,
                   "frame type bits match the MQTT library");
    }
#endif

    // S1-U09: the 1-byte marker the MQTT library sends for an oversize
    // command is rejected by the length check and leaves the sequence
    // state untouched.
    {
        bool b_marker;
        bool b_marker_frame;

        seq_state.last_seq   = (uint8_t) TEST_SEQ_BASE;
        seq_state.b_have_seq = true;

        // The marker itself: one zero byte.
        raw[0] = 0u;
        b_marker = s1_cmd_check(raw, TEST_MARKER_LEN, &seq_state, &packed);

        // A frame that WOULD be valid at 3 bytes, cut to 1 byte. It would be
        // accepted if the length check were missing, so this proves the
        // length check is what rejects it.
        make_cmd(raw, TEST_SEQ_FRESH, PROTO_VERSION, S1_CMD_START, 0u);
        b_marker_frame = s1_cmd_check(raw, TEST_MARKER_LEN, &seq_state,
                                      &packed);

        TEST_CHECK("S1-U09",
                   (false == b_marker) && (false == b_marker_frame)
                   && seq_state.b_have_seq
                   && (TEST_SEQ_BASE == (uint32_t) seq_state.last_seq),
                   "1-byte oversize marker rejected, sequence unchanged");
    }

    TEST_SUMMARY("subsystem1");

    return;
}

#endif /* APP_TEST */

/*** end of file ***/