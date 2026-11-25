
/*
 * Buddy 4 GY-511 / LSM303DLHC physical motion classification.
 *
 * The task is the sole owner of I2C0 and performs all transfers.
 * GP4 = SDA, GP5 = SCL (configured in device/i2c/sysdepend/rp2040).
 *
 * Sensor orientation on robot:
 * X = right
 * Y = rear
 * Z = upwards
 *
 * Sampling interval: approximately 10 ms.
 *
 * Change of requirement (week 12): collision detection removed. Each
 * sample now also runs the lift / overturn detector (buddy4_handling.h)
 * and mirrors the result into flg_s4_imu (IF-13) at once, so the Vehicle
 * Controller can stop the motors within one VC tick.
 */

#include "buddy4_imu_processing.h"
#include "buddy4_imu.h"
#include "buddy4_classifier.h"
#include "buddy4_handling.h"
#include "buddy4_ramp.h"
#include "buddy4_hump.h"
#include "subsystem4.h"
#include "../subsystem2/wheel_encoder.h"

#include <tk/device.h>
#include "../../device/include/dev_i2c.h"
#include "../subsystem1/subsystem1.h"

/* LSM303DLHC accelerometer registers. */
#define ACC_CTRL_REG1_A 0x20u
#define ACC_CTRL_REG4_A 0x23u
#define ACC_OUT_X_L_A 0x28u
#define ACC_WHO_AM_I_A 0x0Fu
#define ACC_WHO_AM_I_VAL 0x33u

/* LSM303DLHC magnetometer registers. */
#define MAG_CRA_REG_M 0x00u
#define MAG_CRB_REG_M 0x01u
#define MAG_MR_REG_M 0x02u
#define MAG_OUT_X_H_M 0x03u
#define MAG_IRA_REG_M 0x0Au
#define MAG_IRB_REG_M 0x0Bu
#define MAG_IRC_REG_M 0x0Cu

#define MAG_ID_A ((UB)'H')
#define MAG_ID_B ((UB)'4')
#define MAG_ID_C ((UB)'3')

/* Accelerometer is configured for 100 Hz. */
#define B4_IMU_SAMPLE_MS 10u

/* 1 = read the 6 accelerometer and 6 magnetometer bytes in one I2C transfer
 * each. 12 single-register reads made one sample take ~55 ms instead of
 * 10 ms. Set to 0 to go back to single-register reads if the IMU faults. */
#define B4_IMU_BURST_READ 1

/* LSM303DLHC accelerometer: set bit 7 of the register address to make it
 * auto-increment over the following registers. */
#define ACC_AUTO_INCREMENT 0x80u

/* Consecutive failed reads (10 ms each) before reconnecting. */
#define B4_IMU_RECONNECT_ERRORS 50u

/* Print diagnostic information approximately once per second. */
#define B4_IMU_PRINT_SAMPLES 500u    /* ~5 s (was 100, ~1 s) */

/*
 * The sensor is mounted with its Z-axis pointing upwards.
 * A stationary, level sensor measures approximately +1000 mg
 * on its Z-axis because of gravity.
 */
#define B4_GRAVITY_MG 1000

/*
 * Hump measurement (buddy4_hump.h): the hump is under the right wheel only,
 * so its height comes from the roll angle. 1 = remove the sideways
 * acceleration of turning (v * omega from the wheel encoders) first.
 */
#define B4_HUMP_TURN_COMP 1

/*
 * 1 = print the hump detector to the serial monitor every
 * B4_HUMP_DEBUG_MS, plus a line when a hump starts or is discarded.
 * Set to 0 for the demo to keep the console quiet.
 */
#if APP_TEST
#define B4_HUMP_DEBUG    0      /* Keep the [TEST] output readable. */
#else
#define B4_HUMP_DEBUG    1
#endif
#define B4_HUMP_DEBUG_MS 100

LOCAL ID b4_i2c_device = -1;
LOCAL UINT b4_flag_bits = S4_FLG_IMU_FAULT;   /* Mirror of flg_s4_imu. */
LOCAL UW b4_accel_address = B4_IMU_ACCEL_ADDR_DEFAULT;
LOCAL uint8_t b4_hump_ok = 0u;
LOCAL uint8_t b4_hump_height_mm = 0u;
LOCAL uint8_t b4_hump_count = 0u;

/* --------------------------------------------------------- */
/* I2C register access                                       */
/* --------------------------------------------------------- */

LOCAL ER b4_read_reg(UW address, UW reg, UB *value)
{
    return i2c_read_reg(b4_i2c_device, address, reg, value);
}

LOCAL ER b4_write_reg(UW address, UW reg, UB value)
{
    return i2c_write_reg(b4_i2c_device, address, reg, value);
}

/* --------------------------------------------------------- */
/* Sensor detection                                          */
/* --------------------------------------------------------- */

LOCAL ER b4_read_accel_id(void)
{
    UB id = 0u;
    ER er;

    er = b4_read_reg(
        B4_IMU_ACCEL_ADDR_DEFAULT,
        ACC_WHO_AM_I_A,
        &id);

    if (er >= E_OK && id == ACC_WHO_AM_I_VAL)
    {
        b4_accel_address = B4_IMU_ACCEL_ADDR_DEFAULT;
        return E_OK;
    }

    id = 0u;

    er = b4_read_reg(
        B4_IMU_ACCEL_ADDR_ALT,
        ACC_WHO_AM_I_A,
        &id);

    if (er >= E_OK && id == ACC_WHO_AM_I_VAL)
    {
        b4_accel_address = B4_IMU_ACCEL_ADDR_ALT;
        return E_OK;
    }

    tm_printf(
        (UB *)"[B4-IMU] accelerometer WHO_AM_I mismatch "
              "(last=0x%02x, er=%d)\n",
        id,
        er);

    return (er < E_OK) ? er : E_IO;
}

LOCAL ER b4_check_mag_id(void)
{
    UB a = 0u;
    UB b = 0u;
    UB c = 0u;
    ER er;

    er = b4_read_reg(B4_IMU_MAG_ADDR, MAG_IRA_REG_M, &a);
    if (er < E_OK)
        return er;

    er = b4_read_reg(B4_IMU_MAG_ADDR, MAG_IRB_REG_M, &b);
    if (er < E_OK)
        return er;

    er = b4_read_reg(B4_IMU_MAG_ADDR, MAG_IRC_REG_M, &c);
    if (er < E_OK)
        return er;

    if (a != MAG_ID_A || b != MAG_ID_B || c != MAG_ID_C)
    {
        tm_printf(
            (UB *)"[B4-IMU] magnetometer ID mismatch: "
                  "%02x %02x %02x\n",
            a, b, c);

        return E_IO;
    }

    return E_OK;
}

/* --------------------------------------------------------- */
/* Sensor configuration                                      */
/* --------------------------------------------------------- */

LOCAL ER b4_configure_sensor(void)
{
    ER er;

    /*
     * Accelerometer:
     * 100 Hz, normal power, X/Y/Z enabled.
     * +/-2 g, high-resolution mode.
     */
    er = b4_write_reg(
        b4_accel_address,
        ACC_CTRL_REG1_A,
        0x57u);

    if (er < E_OK)
        return er;

    er = b4_write_reg(
        b4_accel_address,
        ACC_CTRL_REG4_A,
        0x08u);

    if (er < E_OK)
        return er;

    /*
     * Magnetometer:
     * 15 Hz, +/-1.3 gauss, continuous-conversion mode.
     */
    er = b4_write_reg(
        B4_IMU_MAG_ADDR,
        MAG_CRA_REG_M,
        0x14u);

    if (er < E_OK)
        return er;

    er = b4_write_reg(
        B4_IMU_MAG_ADDR,
        MAG_CRB_REG_M,
        0x20u);

    if (er < E_OK)
        return er;

    er = b4_write_reg(
        B4_IMU_MAG_ADDR,
        MAG_MR_REG_M,
        0x00u);

    return er;
}

/* --------------------------------------------------------- */
/* Raw sensor reading                                        */
/* --------------------------------------------------------- */

LOCAL int16_t b4_make_i16(UB low, UB high)
{
    return (int16_t)((UW)(((UW)high << 8) | (UW)low));
}

#if B4_IMU_BURST_READ
/* Read n consecutive registers in one I2C transfer. */
LOCAL ER b4_read_block(UW address, UW reg, UB *buf, SZ n)
{
    T_I2C_EXEC exec;
    SZ asz;
    UW radr = reg;      /* Low byte is sent, as in i2c_read_reg(). */

    exec.sadr = address;
    exec.snd_size = 1;
    exec.snd_data = (UB *)&radr;
    exec.rcv_size = n;
    exec.rcv_data = buf;

    return tk_swri_dev(b4_i2c_device, TDN_I2C_EXEC, &exec,
                       sizeof(exec), &asz);
}

LOCAL ER b4_read_raw(B4_IMU_RAW *sample)
{
    UB a[6];
    UB m[6];
    ER er;

    /* Accelerometer: X, Y, Z, little-endian. */
    er = b4_read_block(b4_accel_address,
                       ACC_OUT_X_L_A | ACC_AUTO_INCREMENT, a, 6);
    if (er < E_OK)
        return er;

    /* Magnetometer: X, Z, Y, big-endian (pointer auto-increments). */
    er = b4_read_block(B4_IMU_MAG_ADDR, MAG_OUT_X_H_M, m, 6);
    if (er < E_OK)
        return er;

    sample->accel_x = b4_make_i16(a[0], a[1]);
    sample->accel_y = b4_make_i16(a[2], a[3]);
    sample->accel_z = b4_make_i16(a[4], a[5]);

    sample->mag_x = (int16_t)((UW)(((UW)m[0] << 8) | m[1]));
    sample->mag_z = (int16_t)((UW)(((UW)m[2] << 8) | m[3]));
    sample->mag_y = (int16_t)((UW)(((UW)m[4] << 8) | m[5]));

    return E_OK;
}
#else
LOCAL ER b4_read_raw(B4_IMU_RAW *sample)
{
    UB xl, xh, yl, yh, zl, zh;
    UB xh_m, xl_m, zh_m, zl_m, yh_m, yl_m;
    ER er;

    /* Accelerometer output is little-endian: X, Y, Z. */

    er = b4_read_reg(
        b4_accel_address,
        ACC_OUT_X_L_A,
        &xl);

    if (er < E_OK)
        return er;

    er = b4_read_reg(
        b4_accel_address,
        ACC_OUT_X_L_A + 1u,
        &xh);

    if (er < E_OK)
        return er;

    er = b4_read_reg(
        b4_accel_address,
        ACC_OUT_X_L_A + 2u,
        &yl);

    if (er < E_OK)
        return er;

    er = b4_read_reg(
        b4_accel_address,
        ACC_OUT_X_L_A + 3u,
        &yh);

    if (er < E_OK)
        return er;

    er = b4_read_reg(
        b4_accel_address,
        ACC_OUT_X_L_A + 4u,
        &zl);

    if (er < E_OK)
        return er;

    er = b4_read_reg(
        b4_accel_address,
        ACC_OUT_X_L_A + 5u,
        &zh);

    if (er < E_OK)
        return er;

    sample->accel_x = b4_make_i16(xl, xh);
    sample->accel_y = b4_make_i16(yl, yh);
    sample->accel_z = b4_make_i16(zl, zh);

    /* Magnetometer order is X, Z, Y and values are big-endian. */

    er = b4_read_reg(
        B4_IMU_MAG_ADDR,
        MAG_OUT_X_H_M,
        &xh_m);

    if (er < E_OK)
        return er;

    er = b4_read_reg(
        B4_IMU_MAG_ADDR,
        MAG_OUT_X_H_M + 1u,
        &xl_m);

    if (er < E_OK)
        return er;

    er = b4_read_reg(
        B4_IMU_MAG_ADDR,
        MAG_OUT_X_H_M + 2u,
        &zh_m);

    if (er < E_OK)
        return er;

    er = b4_read_reg(
        B4_IMU_MAG_ADDR,
        MAG_OUT_X_H_M + 3u,
        &zl_m);

    if (er < E_OK)
        return er;

    er = b4_read_reg(
        B4_IMU_MAG_ADDR,
        MAG_OUT_X_H_M + 4u,
        &yh_m);

    if (er < E_OK)
        return er;

    er = b4_read_reg(
        B4_IMU_MAG_ADDR,
        MAG_OUT_X_H_M + 5u,
        &yl_m);

    if (er < E_OK)
        return er;

    sample->mag_x = (int16_t)((UW)(((UW)xh_m << 8) | xl_m));

    sample->mag_z = (int16_t)((UW)(((UW)zh_m << 8) | zl_m));

    sample->mag_y = (int16_t)((UW)(((UW)yh_m << 8) | yl_m));

    return E_OK;
}
#endif

/* --------------------------------------------------------- */
/* Motion preprocessing                                      */
/* --------------------------------------------------------- */

/* Calculate absolute difference between two signed values. */
LOCAL int32_t b4_abs_difference(int16_t current, int16_t previous)
{
    int32_t difference;

    difference = (int32_t)current - (int32_t)previous;

    if (difference < 0)
    {
        difference = -difference;
    }

    return difference;
}

/* Keep a calculated value within the int16_t range. */
LOCAL int16_t b4_limit_i16(int32_t value)
{
    if (value > 32767)
        return 32767;

    if (value < -32768)
        return -32768;

    return (int16_t)value;
}
/*
 * Calculate acceleration activity using changes between
 * consecutive calibrated samples.
 *
 * The three axis differences are averaged and then smoothed
 * to reduce the effect of small sensor fluctuations.
 */
LOCAL int16_t b4_calculate_motion_level(
    const B4_ACCEL_MG *current,
    const B4_ACCEL_MG *previous,
    int32_t *filtered_activity)
{
    int32_t dx;
    int32_t dy;
    int32_t dz;
    int32_t activity;

    dx = b4_abs_difference(current->x_mg, previous->x_mg);
    dy = b4_abs_difference(current->y_mg, previous->y_mg);
    dz = b4_abs_difference(current->z_mg, previous->z_mg);

    activity = (dx + dy + dz) / 3;

    /*
     * Simple exponential smoothing:
     * 7/8 previous activity + 1/8 current activity.
     */
    *filtered_activity =
        ((*filtered_activity * 7) + activity) / 8;

    return b4_limit_i16(*filtered_activity);
}

/*
 * Calculate vertical acceleration deviation.
 *
 * The sensor's positive Z-axis points upwards.
 * A level, stationary sensor should measure approximately
 * +1000 mg due to gravity.
 *
 * This is an initial approximation. It is not a complete
 * gravity-compensation or slope-detection algorithm.
 */
LOCAL int16_t b4_calculate_vertical_delta(
    const B4_ACCEL_MG *calibrated)
{
    int32_t delta;

    delta = (int32_t)calibrated->z_mg - B4_GRAVITY_MG;

    return b4_limit_i16(delta);
}

/* --------------------------------------------------------- */
/* IF-13: lift / overturn / fault levels for the VC           */
/* --------------------------------------------------------- */

/*
 * Make flg_s4_imu equal to the given bits. Only this task writes the
 * flag, and only bits that changed are touched, so the VC (which reads
 * with tk_ref_flg) never sees a stale or half-written pattern.
 */
LOCAL void b4_set_flag_bits(UINT bits)
{
    UINT set_bits = bits & ~b4_flag_bits;
    UINT clr_bits = b4_flag_bits & ~bits;

    if (set_bits != 0u)
    {
        (void)tk_set_flg(flg_s4_imu, set_bits);
    }

    if (clr_bits != 0u)
    {
        /* tk_clr_flg keeps the bits that are 1 in its pattern. */
        (void)tk_clr_flg(flg_s4_imu, ~clr_bits);
    }

    if ((set_bits | clr_bits) != 0u)
    {
        tm_printf((UB *)"[B4-IMU] lifted=%u overturned=%u fault=%u\n",
                  (bits & S4_FLG_LIFTED) ? 1u : 0u,
                  (bits & S4_FLG_OVERTURNED) ? 1u : 0u,
                  (bits & S4_FLG_IMU_FAULT) ? 1u : 0u);
    }

    b4_flag_bits = bits;
}

/* --------------------------------------------------------- */
/* MQTT terrain telemetry                                    */
/* --------------------------------------------------------- */

LOCAL void b4_publish_terrain(B4_EVENT event, int32_t lift_peak)
{
    uint8_t frame[PROTO_MAX_FRAME_BYTES];
    proto_bits_t bits;

    proto_init(&bits, frame, sizeof(frame));
    proto_begin(&bits, PROTO_TYPE_TERRAIN);

    /* Existing Buddy 4 event information. */
    proto_put(&bits,
              b4_event_state(event),
              PROTO_TER_STATE_BITS);

    proto_put(&bits,
              b4_event_is_overturned(event),
              PROTO_TER_FLIP_BITS);

    proto_put(&bits,
              b4_event_has_imu_fault(event),
              PROTO_TER_FAULT_BITS);

    proto_put(&bits, b4_hump_ok, PROTO_TER_HUMP_OK_BITS);
    proto_put(&bits, b4_hump_height_mm, PROTO_TER_HUMP_MM_BITS);
    proto_put(&bits, b4_hump_count, PROTO_TER_HUMPS_BITS);

    /* Tuning aid: largest positive lift_acc since the last frame. */
    proto_put_sat(&bits,
                  (lift_peak > 0) ? (uint32_t)(lift_peak / 1000) : 0u,
                  PROTO_TER_LIFT_PEAK_BITS);

    (void)s1_publish(frame, proto_end(&bits), false);
}

/*
 * Sideways acceleration caused by turning, in mg, in the sensor X-right
 * frame. a = v * omega, omega = (v_right - v_left) / track. A left turn
 * (v_right > v_left) accelerates the car to the left, which the sensor
 * reads as -X, so the hump detector ADDS this value to X.
 * Wheel speeds come from the encoder distances, lightly smoothed.
 */
typedef struct {
    float prev_left_cm;
    float prev_right_cm;
    float speed_left_cms;
    float speed_right_cms;
    uint8_t valid;
    uint8_t spinning;       /* 1 = spinning or pivoting (line search). */
} B4_TURN_COMP;

LOCAL int16_t b4_turn_comp_mg(B4_TURN_COMP *tc, int32_t dt_ms)
{
    float left = encoder_getLeftDistance();
    float right = encoder_getRightDistance();
    float dt_s;
    float v;
    float omega;
    float mg;

    if ((tc->valid == 0u) || (dt_ms <= 0))
    {
        tc->prev_left_cm = left;
        tc->prev_right_cm = right;
        tc->speed_left_cms = 0.0f;
        tc->speed_right_cms = 0.0f;
        tc->valid = 1u;
        tc->spinning = 0u;
        return 0;
    }

    dt_s = (float)dt_ms / 1000.0f;

    /* Exponential smoothing: 3/4 old + 1/4 new. */
    tc->speed_left_cms += (((left - tc->prev_left_cm) / dt_s)
                           - tc->speed_left_cms) * 0.25f;
    tc->speed_right_cms += (((right - tc->prev_right_cm) / dt_s)
                            - tc->speed_right_cms) * 0.25f;
    tc->prev_left_cm = left;
    tc->prev_right_cm = right;

    /*
     * Spinning or pivoting: the wheels differ by more than the car's
     * forward speed (turn radius under half the track), plus 1 cm/s so
     * that standing still does not count.
     */
    {
        float diff = tc->speed_right_cms - tc->speed_left_cms;
        float fwd = tc->speed_right_cms + tc->speed_left_cms;

        if (diff < 0.0f) diff = -diff;
        if (fwd < 0.0f)  fwd = -fwd;
        tc->spinning = (diff > (fwd + 1.0f)) ? 1u : 0u;
    }

#if B4_HUMP_TURN_COMP
    v = (tc->speed_left_cms + tc->speed_right_cms) * 0.5f;
    omega = (tc->speed_right_cms - tc->speed_left_cms)
          / ((float)B4_HUMP_TRACK_MM / 10.0f);
    mg = v * omega * (1000.0f / 981.0f);      /* cm/s^2 -> mg */

    if (mg > 500.0f)  mg = 500.0f;
    if (mg < -500.0f) mg = -500.0f;
    return (int16_t)mg;
#else
    (void)v;
    (void)omega;
    (void)mg;
    return 0;
#endif
}

LOCAL int32_t b4_right_travel_mm(void)
{
    return (int32_t)(encoder_getRightDistance() * 10.0f);
}

/* --------------------------------------------------------- */
/* Connect: open I2C, find and configure the IMU (retries)   */
/* --------------------------------------------------------- */

/*
 * Returns only when the IMU answers and is configured. Until then it
 * reports the reason on the console and a fault terrain frame every 2 s.
 */
LOCAL void b4_imu_connect(void)
{
    ER er;

    /*
     * Find and configure the IMU. If anything fails, report it (console
     * every 2 s, terrain frame with the fault bit) and try again, instead
     * of ending the task: a loose wire or a late power-up then recovers,
     * and the failure stays visible after the boot messages are gone.
     */
    for (;;)
    {
        if (b4_i2c_device < E_OK)
        {
            b4_i2c_device = tk_opn_dev((const UB *)"iica", TD_UPDATE);
        }

        if (b4_i2c_device < E_OK)
        {
            er = b4_i2c_device;
            tm_printf((UB *)"[B4-IMU] cannot open iica: %d\n", er);
        }
        else if ((er = b4_read_accel_id()) < E_OK)
        {
            tm_printf((UB *)"[B4-IMU] accelerometer not detected: %d "
                            "(check SDA=GP4, SCL=GP5, 3V3, GND)\n", er);
        }
        else if ((er = b4_check_mag_id()) < E_OK)
        {
            tm_printf((UB *)"[B4-IMU] magnetometer not detected: %d\n", er);
        }
        else if ((er = b4_configure_sensor()) < E_OK)
        {
            tm_printf((UB *)"[B4-IMU] sensor configuration failed: %d\n",
                      er);
        }
        else
        {
            break;                          /* IMU ready. */
        }

        b4_publish_terrain(b4_pack_event(B4_STATE_FAULT, 0u, 1u), 0);
        tk_dly_tsk(2000u);
    }

    tm_printf((UB *)"[B4-IMU] I2C device id=%d\n", (INT)b4_i2c_device);
}

/* --------------------------------------------------------- */
/* Physical motion classification                            */
/* --------------------------------------------------------- */

EXPORT void b4_imu_task(INT stacd, void *exinf)
{
    B4_ACCEL_MG accel_mg;
    B4_ACCEL_MG calibrated_mg;
    B4_ACCEL_MG previous_mg;

    B4_IMU_RAW sample;
    B4_MOTION_INPUT motion_input;
    B4_EVENT event;
    B4_EVENT previous_event = 0u;
    int32_t terrain_ms = 0;
    int32_t terrain_peak = 0;
    uint8_t terrain_first_publish = 1u;
    B4_HANDLING handling;
    UINT flag_bits;
    SYSTIM now_tm;
    UW last_ms = 0u;
    int32_t dt_ms = (int32_t)B4_IMU_SAMPLE_MS;
    int32_t lift_peak = 0;      /* Largest lift_acc since the last print. */
    int32_t dt_peak = 0;        /* Longest sample interval since then. */
    /* static: keep ~280 bytes of hump state off the 2 KB task stack. */
    static B4_HUMP hump;
    static B4_TURN_COMP turn_comp = {0.0f, 0.0f, 0.0f, 0.0f, 0u, 0u};
    int16_t hump_lat_mg = 0;
    int32_t hump_travel_mm = 0;
#if B4_HUMP_DEBUG
    int32_t hump_dbg_ms = 0;
    uint8_t hump_was_ready = 0u;
#endif

    ER er;

    int32_t filtered_activity = 0;
    uint16_t print_counter = 0u;

    uint8_t previous_valid = 0u;
    uint8_t read_errors = 0u;

    (void)stacd;
    (void)exinf;

    tm_printf(
        (UB *)"[B4-IMU] Physical motion classification task starting\n");

    b4_imu_connect();

    tm_printf(
        (UB *)"[B4-IMU] LSM303DLHC detected; accel address=0x%02x\n",
        b4_accel_address);

    tm_printf(
        (UB *)"[B4-IMU] Sampling every %u ms\n",
        B4_IMU_SAMPLE_MS);

    /* Sensor found: clear IMU_FAULT, start the handling detector. */
    b4_handling_init(&handling);
    b4_hump_init(&hump);
    b4_set_flag_bits(0u);

    /*
     * Main sensor sampling and classification loop.
     */
    while (1)
    {
        /* Real time since the last sample: the detector works in ms. */
        (void)tk_get_otm(&now_tm);
        if (last_ms != 0u)
        {
            dt_ms = (int32_t)(now_tm.lo - last_ms);
        }
        last_ms = now_tm.lo;
        if (dt_ms > dt_peak)
        {
            dt_peak = dt_ms;
        }

        er = b4_read_raw(&sample);

        if (er >= E_OK)
        {
            read_errors = 0u;

            /* Step 1: Convert raw accelerometer readings into mg. */
            b4_accel_convert(&sample, &accel_mg);

            /* Step 2: Apply six-position calibration. */
            b4_accel_calibrate(&accel_mg, &calibrated_mg);

            /*
             * Step 3: Prepare motion classifier input.
             */
            motion_input.imu_fault = 0u;
            /* Pass calibrated acceleration readings to classifier. */
            motion_input.accel_x_mg = calibrated_mg.x_mg;
            motion_input.accel_y_mg = calibrated_mg.y_mg;
            motion_input.accel_z_mg = calibrated_mg.z_mg;

            /*
             * The LSM303DLHC has no gyroscope.
             * Do not invent a yaw-rate measurement.
             */
            motion_input.yaw_rate_mdps = 0;

            if (previous_valid == 0u)
            {
                /*
                 * First valid sample establishes the reference
                 * for calculating acceleration changes.
                 */
                previous_mg = calibrated_mg;
                previous_valid = 1u;

                motion_input.motion_level_mg = 0;
            }
            else
            {
                /* Calculate smoothed acceleration activity. */
                motion_input.motion_level_mg =
                    b4_calculate_motion_level(
                        &calibrated_mg,
                        &previous_mg,
                        &filtered_activity);

                /* Save the current sample for the next iteration. */
                previous_mg = calibrated_mg;
            }
            /*
             * Calculate vertical acceleration deviation.
             * Positive Z is upwards.
             */
            motion_input.vertical_delta_mg =
                b4_calculate_vertical_delta(&calibrated_mg);

            /*
             * Step 4: Lift / overturn detection (replaces collision
             * detection). Tell the VC at once, before anything else.
             */
            b4_handling_update_dt(&handling,
                                  calibrated_mg.x_mg,
                                  calibrated_mg.y_mg,
                                  calibrated_mg.z_mg,
                                  dt_ms);

            /*
             * Hump under the right wheel: height from the roll angle.
             * Needs ~0.6 s on flat ground after start-up to learn level.
             */
            hump_lat_mg = b4_turn_comp_mg(&turn_comp, dt_ms);
            hump_travel_mm = b4_right_travel_mm();
            hump.hold = turn_comp.spinning;

            if (b4_hump_update(&hump,
                               calibrated_mg.x_mg,
                               calibrated_mg.y_mg,
                               calibrated_mg.z_mg,
                               hump_lat_mg,
                               hump_travel_mm,
                               dt_ms) != 0u)
            {
                int32_t mm = (hump.result_dmm + 5) / 10;

                b4_hump_ok = 1u;
                b4_hump_height_mm = (mm > 255) ? 255u
                                  : (mm < 0)   ? 0u
                                  : (uint8_t)mm;
                if (b4_hump_count < 15u)
                {
                    b4_hump_count++;
                }

                tm_printf((UB *)"[B4-IMU] HUMP #%u: height=%d.%d mm "
                                "(pitch check %d.%d mm), right wheel "
                                "travel=%d mm\n",
                          (UINT)b4_hump_count,
                          (INT)(hump.result_dmm / 10),
                          (INT)(hump.result_dmm % 10),
                          (INT)(hump.result_pitch_dmm / 10),
                          (INT)((hump.result_pitch_dmm < 0)
                                ? -(hump.result_pitch_dmm % 10)
                                : (hump.result_pitch_dmm % 10)),
                          (INT)hump.result_travel_mm);

                /* Send the new height at once. */
                terrain_first_publish = 1u;
            }

#if B4_HUMP_DEBUG
            if ((hump.ready != 0u) && (hump_was_ready == 0u))
            {
                tm_printf((UB *)"[HUMP] level learned: X=%d Y=%d Z=%d mg "
                                "(keep the car still on flat ground at "
                                "power-up)\n",
                          calibrated_mg.x_mg, calibrated_mg.y_mg,
                          calibrated_mg.z_mg);
            }
            hump_was_ready = hump.ready;

            if (hump.started != 0u)
            {
                tm_printf((UB *)"[HUMP] start: roll height=%d x0.1mm "
                                "travel=%d mm\n",
                          (INT)hump.now_dmm, (INT)hump_travel_mm);
            }

            if (hump.rejected != B4_HUMP_REJ_NONE)
            {
                tm_printf((UB *)"[HUMP] discarded: %s (peak=%d x0.1mm, "
                                "right wheel moved %d mm)\n",
                          (hump.rejected == B4_HUMP_REJ_LOW_PEAK)
                              ? "peak below 6 mm"
                          : (hump.rejected == B4_HUMP_REJ_NO_TRAVEL)
                              ? "right wheel moved < 30 mm"
                              : "tilted > 10 s or rolled > 40 cm",
                          (INT)hump.reject_peak_dmm,
                          (INT)hump.reject_travel_mm);
            }

            hump_dbg_ms += dt_ms;
            if (hump_dbg_ms >= B4_HUMP_DEBUG_MS)
            {
                hump_dbg_ms = 0;
                /* h = roll height (used), p = pitch check, both x0.1 mm.
                   dx/dy = X/Y minus level (mg); lat = turn correction. */
                tm_printf((UB *)"[HUMP] rdy=%u act=%u hold=%u h=%d p=%d | "
                                "X=%d Y=%d Z=%d dx=%d dy=%d lat=%d | "
                                "R=%d mm dt=%d\n",
                          (UINT)hump.ready, (UINT)hump.active,
                          (UINT)hump.hold,
                          (INT)hump.now_dmm, (INT)hump.now_pitch_dmm,
                          calibrated_mg.x_mg, calibrated_mg.y_mg,
                          calibrated_mg.z_mg,
                          (INT)hump.now_dev_x_mg, (INT)hump.now_dev_y_mg,
                          (INT)hump_lat_mg, (INT)hump_travel_mm,
                          (INT)dt_ms);
            }
#endif

            if (handling.lift_acc > lift_peak)
            {
                lift_peak = handling.lift_acc;
            }
            if (handling.lift_acc > terrain_peak)
            {
                terrain_peak = handling.lift_acc;
            }

            motion_input.lifted = handling.b_lifted;
            motion_input.overturned = handling.b_overturned;

            flag_bits = 0u;
            if (handling.b_lifted != 0u)
            {
                flag_bits |= S4_FLG_LIFTED;
            }
            if (handling.b_overturned != 0u)
            {
                flag_bits |= S4_FLG_OVERTURNED;
            }
            b4_set_flag_bits(flag_bits);

            /*
             * Step 5: Classify the processed sensor readings.
             */
            event = b4_classify_motion(motion_input);
        }
        else
        {
            /*
             * Sensor communication failed.
             * Report an IMU fault instead of classifying
             * invalid sensor data.
             */
            motion_input.motion_level_mg = 0;
            motion_input.vertical_delta_mg = 0;
            motion_input.accel_x_mg = 0;
            motion_input.accel_y_mg = 0;
            motion_input.accel_z_mg = 0;
            motion_input.yaw_rate_mdps = 0;
            motion_input.imu_fault = 1u;
            motion_input.lifted = 0u;
            motion_input.overturned = 0u;

            /*
             * Keep any LIFTED / OVERTURNED latch (the VC has already
             * stopped), forget the integrator, and report the fault.
             */
            b4_handling_reset_integrator(&handling);
            b4_hump_abort(&hump);
            turn_comp.valid = 0u;
            b4_set_flag_bits((b4_flag_bits & (S4_FLG_LIFTED |
                                              S4_FLG_OVERTURNED)) |
                             S4_FLG_IMU_FAULT);

            event = b4_classify_motion(motion_input);

            /*
             * Reset previous-sample tracking.
             * This prevents a large artificial difference
             * after sensor communication recovers.
             */
            previous_valid = 0u;
            filtered_activity = 0;

            /*
             * Persistent failure (0.5 s): print why, drop the device
             * and connect again. E_ID (-18) means the device descriptor
             * itself is no longer valid, so it must be reopened.
             */
            if (read_errors < 255u)
            {
                read_errors++;
            }

            if (read_errors >= B4_IMU_RECONNECT_ERRORS)
            {
                tm_printf((UB *)"[B4-IMU] %u read errors (last %d, "
                                "dev id %d): reconnecting\n",
                          (UINT)read_errors, er, (INT)b4_i2c_device);

                if (er != E_ID)
                {
                    (void)tk_cls_dev(b4_i2c_device, 0);
                }
                b4_i2c_device = -1;
                read_errors = 0u;
                b4_imu_connect();
            }
        }

        /* ----------------------------------------------------- */
        /* Step 6: Publish terrain telemetry to MQTT.            */
        /* Publish on event change or approximately every 1 sec. */
        /* ----------------------------------------------------- */

        terrain_ms += dt_ms;

        if ((terrain_first_publish != 0u) ||
            (event != previous_event) ||
            (terrain_ms >= 1000))
        {
            b4_publish_terrain(event, terrain_peak);

            previous_event = event;
            terrain_ms = 0;
            terrain_peak = 0;
            terrain_first_publish = 0u;
        }

        /*
         * Step 7: Limit serial output.
         * Sensor sampling and classification continue every loop.
         */
        print_counter++;

        if (print_counter >= B4_IMU_PRINT_SAMPLES)
        {
            print_counter = 0u;

            /* Tuning (S4-M01): B4_LIFT_TRIGGER should be at least 1.5 x
               the largest peak seen while driving normally. */
            tm_printf((UB *)"[B4-IMU] lift peak=%d (trigger %d) | "
                            "longest loop=%d ms\n",
                      (INT)lift_peak, (INT)B4_LIFT_TRIGGER, (INT)dt_peak);

            if (er >= E_OK)
            {
                tm_printf(
                    (UB *)"[B4-IMU] "
                          "CAL X=%d Y=%d Z=%d mg | "
                          "Motion=%d mg | "
                          "Lift acc=%d Peak=%d | "
                          "Lifted=%u Overturned=%u | "
                          "Vertical delta=%d mg | "
                          "Hump now=%d (pitch %d) x0.1 mm | "
                          "Event=0x%02x | State=%u\n",
                    calibrated_mg.x_mg,
                    calibrated_mg.y_mg,
                    calibrated_mg.z_mg,
                    motion_input.motion_level_mg,
                    (INT)handling.lift_acc,
                    (INT)lift_peak,
                    (UINT)handling.b_lifted,
                    (UINT)handling.b_overturned,
                    motion_input.vertical_delta_mg,
                    (INT)hump.now_dmm,
                    (INT)hump.now_pitch_dmm,
                    (UB)event,
                    (UB)b4_event_state(event));
            }
            else
            {
                tm_printf(
                    (UB *)"[B4-IMU] Sensor read error=%d | "
                          "Event=0x%02x | State=%u\n",
                    er,
                    (UB)event,
                    (UB)b4_event_state(event));
            }
            lift_peak = 0;
            dt_peak = 0;
        }

        /* Wait before taking the next sample. */
        tk_dly_tsk(B4_IMU_SAMPLE_MS);
    }
}