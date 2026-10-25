/** @file subsystem1.c
 *
 * @brief Subsystem 1 (Comms): creates kernel objects, starts comms_task.
 */
#include "subsystem1.h"

#if TM_MQTT
#include "mqtt_utk.h"
#endif

#define WILL_SEQ        (0xFFu)
#define ALL_ONES_16     (0xFFFFu)
#define ALL_ONES_8      (0xFFu)

EXPORT ID                g_mbf_telemetry   = 0;
EXPORT ID                g_mbf_cmd_raw     = 0;
EXPORT ID                g_mbf_remote_cmd  = 0;
EXPORT ID                g_flg_remote      = 0;
EXPORT ID                g_flg_comms       = 0;
EXPORT volatile uint8_t  g_s1_fault_bits   = 0u;

#if TM_MQTT
static uint8_t g_will_frame[PROTO_MAX_FRAME_BYTES + 1u];    // +1: NUL.
#endif

static ID create_mbf(uint32_t buf_bytes, uint32_t max_msg);
static ID create_flg(void);
#if TM_MQTT
static uint32_t build_will_frame(void);
#endif

ER
subsystem1_init (void)
{
    ER err = E_OK;

    g_mbf_telemetry  = create_mbf(S1_TLM_BUF_BYTES, S1_TLM_MAX_MSG);
    g_mbf_cmd_raw    = create_mbf(S1_RAW_BUF_BYTES, S1_RAW_MAX_MSG);
    g_mbf_remote_cmd = create_mbf(S1_CMD_BUF_BYTES, S1_CMD_MAX_MSG);
    g_flg_remote     = create_flg();
    g_flg_comms      = create_flg();

    ID const ids[] = {g_mbf_telemetry, g_mbf_cmd_raw, g_mbf_remote_cmd,
                      g_flg_remote, g_flg_comms};

    for (uint32_t idx = 0u; idx < (sizeof(ids) / sizeof(ids[0])); idx++)
    {
        if ((E_OK == err) && (ids[idx] < E_OK))
        {
            err = ids[idx];
        }
    }

#if TM_MQTT
    if (E_OK == err)
    {
        uint32_t will_len = build_will_frame();

        mqtt_utk_bind((int32_t) g_mbf_telemetry, (int32_t) g_mbf_cmd_raw,
                      g_will_frame, will_len);
    }
#endif

    return (err);
}

ER
subsystem1_start (void)
{
    // Core 0 (TP_PRC1): same core as the radio service, leaving core 1 free
    // for the time-critical tasks.
    ID tid = app_start_task(comms_task, S1_PRI_COMMS, TP_PRC1, "comms_task");

    return ((tid < E_OK) ? tid : E_OK);
}

static ID
create_mbf (uint32_t buf_bytes, uint32_t max_msg)
{
    T_CMBF cmbf = {0};

    // NOTE: Both sizes are small constants, so the casts cannot truncate.
    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = (SZ) buf_bytes;
    cmbf.maxmsz = (INT) max_msg;

    return (tk_cre_mbf(&cmbf));
}

static ID
create_flg (void)
{
    T_CFLG cflg = {0};

    cflg.flgatr  = TA_TFIFO | TA_WSGL;
    cflg.iflgptn = 0u;

    return (tk_cre_flg(&cflg));
}

#if TM_MQTT
static uint32_t
build_will_frame (void)
{
    proto_bits_t bits;

    proto_init(&bits, g_will_frame, sizeof(g_will_frame));
    proto_begin(&bits, PROTO_TYPE_HEARTBEAT);
    proto_put(&bits, S1_LINK_OFFLINE, PROTO_HB_LINK_BITS);
    proto_put(&bits, ALL_ONES_16, PROTO_HB_UPTIME_BITS);
    proto_put(&bits, ALL_ONES_8, PROTO_HB_DROP_BITS);
    proto_put(&bits, ALL_ONES_8, PROTO_HB_CMD_OK_BITS);
    proto_put(&bits, ALL_ONES_8, PROTO_HB_CMD_BAD_BITS);

    // NOTE: lwIP treats the will payload as a C string, so no byte may be 0.
    // Seq is written as 0 by proto_begin, so set it to 0xFF. With these
    // field values every byte of the 8-byte frame is then non-zero.
    g_will_frame[0] = WILL_SEQ;

    return (proto_end(&bits));
}
#endif

/*** end of file ***/