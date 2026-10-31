
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
 * Classification is currently for standalone testing.
 */

#include "buddy4_imu_processing.h"
#include "buddy4_imu.h"
#include "buddy4_classifier.h"

#include <tk/device.h>
#include "../../device/include/dev_i2c.h"

/* LSM303DLHC accelerometer registers. */
#define ACC_CTRL_REG1_A       0x20u
#define ACC_CTRL_REG4_A       0x23u
#define ACC_OUT_X_L_A         0x28u
#define ACC_WHO_AM_I_A        0x0Fu
#define ACC_WHO_AM_I_VAL      0x33u

/* LSM303DLHC magnetometer registers. */
#define MAG_CRA_REG_M         0x00u
#define MAG_CRB_REG_M         0x01u
#define MAG_MR_REG_M          0x02u
#define MAG_OUT_X_H_M         0x03u
#define MAG_IRA_REG_M         0x0Au
#define MAG_IRB_REG_M         0x0Bu
#define MAG_IRC_REG_M         0x0Cu

#define MAG_ID_A              ((UB)'H')
#define MAG_ID_B              ((UB)'4')
#define MAG_ID_C              ((UB)'3')

/* Accelerometer is configured for 100 Hz. */
#define B4_IMU_SAMPLE_MS      10u

/* Print diagnostic information approximately once per second. */
#define B4_IMU_PRINT_SAMPLES  100u

/*
 * The sensor is mounted with its Z-axis pointing upwards.
 * A stationary, level sensor measures approximately +1000 mg
 * on its Z-axis because of gravity.
 */
#define B4_GRAVITY_MG         1000

LOCAL ID b4_i2c_device = -1;
LOCAL UW b4_accel_address = B4_IMU_ACCEL_ADDR_DEFAULT;


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
    return (int16_t)(
        (UW)(((UW)high << 8) | (UW)low));
}


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

    sample->mag_x = (int16_t)(
        (UW)(((UW)xh_m << 8) | xl_m));

    sample->mag_z = (int16_t)(
        (UW)(((UW)zh_m << 8) | zl_m));

    sample->mag_y = (int16_t)(
        (UW)(((UW)yh_m << 8) | yl_m));

    return E_OK;
}


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

    ER er;

    int32_t filtered_activity = 0;
    uint16_t print_counter = 0u;

    uint8_t previous_valid = 0u;

    (void)stacd;
    (void)exinf;

    tm_printf(
        (UB *)"[B4-IMU] Physical motion classification task starting\n");

    /* Open I2C device. */
    b4_i2c_device = tk_opn_dev(
        (const UB *)"iica",
        TD_UPDATE);

    if (b4_i2c_device < E_OK)
    {
        tm_printf(
            (UB *)"[B4-IMU] cannot open iica: %d\n",
            b4_i2c_device);

        return;
    }

    /* Check accelerometer identity. */
    er = b4_read_accel_id();

    if (er < E_OK)
    {
        tm_printf(
            (UB *)"[B4-IMU] accelerometer not detected: %d\n",
            er);

        tk_cls_dev(b4_i2c_device, 0);
        return;
    }

    /* Check magnetometer identity. */
    er = b4_check_mag_id();

    if (er < E_OK)
    {
        tm_printf(
            (UB *)"[B4-IMU] magnetometer not detected: %d\n",
            er);

        tk_cls_dev(b4_i2c_device, 0);
        return;
    }

    /* Configure both sensor components. */
    er = b4_configure_sensor();

    if (er < E_OK)
    {
        tm_printf(
            (UB *)"[B4-IMU] sensor configuration failed: %d\n",
            er);

        tk_cls_dev(b4_i2c_device, 0);
        return;
    }

    tm_printf(
        (UB *)"[B4-IMU] LSM303DLHC detected; accel address=0x%02x\n",
        b4_accel_address);

    tm_printf(
        (UB *)"[B4-IMU] Sampling every %u ms\n",
        B4_IMU_SAMPLE_MS);

    /*
     * Main sensor sampling and classification loop.
     */
    while (1)
    {
        er = b4_read_raw(&sample);

        if (er >= E_OK)
        {
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
             * Step 4: Classify the processed sensor readings.
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

            event = b4_classify_motion(motion_input);

            /*
             * Reset previous-sample tracking.
             * This prevents a large artificial difference
             * after sensor communication recovers.
             */
            previous_valid = 0u;
            filtered_activity = 0;
        }

        /*
         * Step 5: Limit serial output.
         * Sensor sampling and classification continue every loop.
         */
        print_counter++;

        if (print_counter >= B4_IMU_PRINT_SAMPLES)
        {
            print_counter = 0u;

            if (er >= E_OK)
            {
                tm_printf(
                    (UB *)"[B4-IMU] "
                           "CAL X=%d Y=%d Z=%d mg | "
                           "Motion=%d mg | "
                           "Vertical delta=%d mg | "
                           "Event=0x%02x | State=%u\n",
                    calibrated_mg.x_mg,
                    calibrated_mg.y_mg,
                    calibrated_mg.z_mg,
                    motion_input.motion_level_mg,
                    motion_input.vertical_delta_mg,
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
        }

        /* Wait before taking the next sample. */
        tk_dly_tsk(B4_IMU_SAMPLE_MS);
    }
}
