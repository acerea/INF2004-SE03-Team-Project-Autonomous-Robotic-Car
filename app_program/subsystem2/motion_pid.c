/*
 * motion_pid.c
 * Encoder-based speed control, distance movement and in-place turning.
 * Updated periodically by the RTOS motion task.
 */

#include <stddef.h>

#include "motion_pid.h"
#include "pid_control.h"
#include "wheel_encoder.h"
#include "motion_control.h"

/* Controller state */

/* Independent speed controllers for each wheel. */
static SpeedPID left_pid;
static SpeedPID right_pid;

/* Encoder counts from the previous update */
static int32_t left_previous_count;
static int32_t right_previous_count;

/* Desired forward speed in cm/s */
static float target_speed = 0.0f;
static int pid_active = 0;

/* FR6: Distance-based movement */
static float left_start_distance = 0.0f;
static float right_start_distance = 0.0f;
static float target_distance = 0.0f;
static int distance_mode = 0;
static int motion_complete = 0;
static int line_correction_pwm = 0;
static int line_error_filtered = 0;

/* FR5: Preliminary straight-line correction gain */
#define STRAIGHT_KP 0.1f

/* Wheel speed PID. Starting (feed-forward) PWM = MIN + PER_CMS x target:
 * the motors barely turn below ~15 %, and ~20 % gave 6-7 cm/s on the
 * floor (log 2026-11-24). KI was 1.0: too slow to pull the speed down. */
#define SPEED_KP              2.0f
#define SPEED_KI              3.0f
#define SPEED_KD              0.01f
#define SPEED_FF_MIN_PWM      12.0f
#define SPEED_FF_PWM_PER_CMS  1.5f

/* Line steering strength (see motion_pidSetLineCorrection). */
#define LINE_GAIN_PCT 2      /* was 3: oversteered on curves at 30 mm/s */
#define LINE_MAX_PWM  15     /* was 30: base PWM at 30 mm/s is only ~14 */

/* FR6: Encoder-based turning */
#define WHEEL_TRACK_CM 9.9f
#define PI_VALUE 3.14159265f
#define TURN_PWM 25

static int turn_mode = 0;
static int turn_direction = 0;  /* -1 = left, +1 = right */

/* Stall help for turns: a wheel that has moved less than TURN_STALL_CM in
 * TURN_CHECK_S gets TURN_BOOST_STEP more PWM, up to TURN_PWM_MAX. At 25 %
 * one wheel alone could not drag the car round once the other had stopped. */
#define TURN_PWM_MAX     50
#define TURN_BOOST_STEP  5
#define TURN_CHECK_S     0.2f
#define TURN_STALL_CM    0.1f
static int   turn_boost_left = 0;
static int   turn_boost_right = 0;
static float turn_check_left = 0.0f;
static float turn_check_right = 0.0f;
static float turn_check_s = 0.0f;
static float turn_target_cm = 0.0f;
static float turn_left_start = 0.0f;
static float turn_right_start = 0.0f;


/* Initialisation and speed commands */

/* Initialise controllers and action flags. */
void motion_pidInit(void)
{
    /* Preliminary gains from recorded PID tests */
    speed_pid_init(&left_pid, 2.0f, 1.0f, 0.01f, 30.0f);
    speed_pid_init(&right_pid, 2.0f, 1.0f, 0.01f, 30.0f);

    left_previous_count = encoder_getLeftCount();
    right_previous_count = encoder_getRightCount();

    target_speed = 0.0f;
    pid_active = 0;
    distance_mode = 0;
    motion_complete = 0;
    turn_mode = 0;
    line_correction_pwm = 0;
    line_error_filtered = 0;
}

/* Set the target speed for forward movement */
void motion_pidSetTargetSpeed(float speed_cm_s)
{
    if (speed_cm_s <= 0.0f) {
        motion_pidStop();
        return;
    }

    /* Reset PID state for a new movement. The starting PWM now depends on
     * the speed asked for (it was a fixed 30 %, far above what slow speeds
     * need: at 3 cm/s the car ran at 6-7 cm/s while the PID crept down). */
    {
        float ff = SPEED_FF_MIN_PWM + SPEED_FF_PWM_PER_CMS * speed_cm_s;

        if (ff > 100.0f) ff = 100.0f;
        speed_pid_init(&left_pid, SPEED_KP, SPEED_KI, SPEED_KD, ff);
        speed_pid_init(&right_pid, SPEED_KP, SPEED_KI, SPEED_KD, ff);
    }

    left_previous_count = encoder_getLeftCount();
    right_previous_count = encoder_getRightCount();

    target_speed = speed_cm_s;
    pid_active = 1;
    distance_mode = 0;
    motion_complete = 0;
}

void motion_pidSetLineCorrection(int line_pos)
{
    int correction;

    if (line_pos > 1000) line_pos = 1000;
    if (line_pos < -1000) line_pos = -1000;

    /* Smooth the IR position so small sensor edges do not jerk the wheels.
     * LINE_GAIN_PCT PWM per 100 position units, capped at LINE_MAX_PWM.
     * Was 8 / 80: with sensors that see the line one at a time the
     * position jumps between about -1000 and +1000, and +/-80 PWM on a
     * ~30 PWM base swung the car from side to side. Tune these two. */
    line_error_filtered = (line_error_filtered * 3 + line_pos) / 4;
    correction = (line_error_filtered * LINE_GAIN_PCT) / 100;
    if (correction > LINE_MAX_PWM) correction = LINE_MAX_PWM;
    if (correction < -LINE_MAX_PWM) correction = -LINE_MAX_PWM;
    line_correction_pwm = correction;
}
/* Change speed without cancelling an active distance target. */
void motion_pidChangeSpeed(float speed_cm_s)
{
    if (speed_cm_s <= 0.0f) {
        motion_pidStop();
        return;
    }

    if (turn_mode) {
        return;
    }

    if (distance_mode) {
        target_speed = speed_cm_s;
    } else if (pid_active && (speed_cm_s == target_speed)) {
        /* The vehicle polls obstacle state continuously.  Do not reset PID
         * state every 10 ms when the requested cruise speed is unchanged. */
        return;
    } else {
        motion_pidSetTargetSpeed(speed_cm_s);
    }
}

/* Periodic control update */

/* Handle turning first, then forward distance/speed control. */
void motion_pidUpdate(float dt_seconds)
{
    if (turn_mode) {
        float left_travelled =
            encoder_getLeftDistance() - turn_left_start;

        float right_travelled =
            encoder_getRightDistance() - turn_right_start;

        /* Convert to absolute distances */
        if (left_travelled < 0.0f)
            left_travelled = -left_travelled;

        if (right_travelled < 0.0f)
            right_travelled = -right_travelled;

        /* The car's rotation follows the AVERAGE of the two wheels. Both
         * keep driving until that reaches the target. (Stopping each wheel
         * on its own target left the slower one to pivot the car alone,
         * and at TURN_PWM it stalled: the turn never completed.) */
        if ((left_travelled + right_travelled) * 0.5f >= turn_target_cm) {
            motion_pidStop();
            motion_complete = 1;
            return;
        }

        /* Stall help: more PWM for a wheel that is not moving. */
        if (dt_seconds > 0.0f) {
            turn_check_s += dt_seconds;
        }
        if (turn_check_s >= TURN_CHECK_S) {
            if ((left_travelled - turn_check_left) < TURN_STALL_CM
                && (TURN_PWM + turn_boost_left) < TURN_PWM_MAX) {
                turn_boost_left += TURN_BOOST_STEP;
            }
            if ((right_travelled - turn_check_right) < TURN_STALL_CM
                && (TURN_PWM + turn_boost_right) < TURN_PWM_MAX) {
                turn_boost_right += TURN_BOOST_STEP;
            }
            turn_check_left = left_travelled;
            turn_check_right = right_travelled;
            turn_check_s = 0.0f;
        }

        motion_setWheelPWM(turn_direction * (TURN_PWM + turn_boost_left),
                           -turn_direction * (TURN_PWM + turn_boost_right));

        return;
    }

    if (!pid_active || dt_seconds <= 0.0f) {
        return;
    }

    float distance_correction = 0.0f;

    if (distance_mode) {
        float left_travelled =
            encoder_getLeftDistance() - left_start_distance;

        float right_travelled =
            encoder_getRightDistance() - right_start_distance;

        float travelled =
            (left_travelled + right_travelled) / 2.0f;

        /* Slow the leading wheel and boost the trailing wheel. */
        distance_correction = 2.0f * (left_travelled - right_travelled);
        if (distance_correction > 5.0f) distance_correction = 5.0f;
        if (distance_correction < -5.0f) distance_correction = -5.0f;

        if (travelled >= target_distance) {
            motion_pidStop();
            motion_complete = 1;
            return;
        }
    }

    int32_t left_now = encoder_getLeftCount();
    int32_t right_now = encoder_getRightCount();

    int32_t left_change = left_now - left_previous_count;
    int32_t right_change = right_now - right_previous_count;

    left_previous_count = left_now;
    right_previous_count = right_now;

    float left_speed =
        encoder_calculateLeftSpeed(left_change, dt_seconds);

    float right_speed =
        encoder_calculateRightSpeed(right_change, dt_seconds);

    float left_pwm =
        speed_pid_update(&left_pid, target_speed,
                         left_speed, dt_seconds);

    float right_pwm =
        speed_pid_update(&right_pid, target_speed,
                         right_speed, dt_seconds);

    /* Correct difference between wheel speeds */
    float speed_error = left_speed - right_speed;
    float correction = STRAIGHT_KP * speed_error + distance_correction;

    /* Slow the faster wheel and boost the slower wheel */
    left_pwm -= correction;
    right_pwm += correction;

    /* Positive line_pos means the line is right; turn right by slowing
     * the right wheel and keeping the left wheel faster. */
    left_pwm += (float)line_correction_pwm;
    right_pwm -= (float)line_correction_pwm;

    /* Limit PWM to forward range */
    if (left_pwm < 0.0f) left_pwm = 0.0f;
    if (left_pwm > 100.0f) left_pwm = 100.0f;

    if (right_pwm < 0.0f) right_pwm = 0.0f;
    if (right_pwm > 100.0f) right_pwm = 100.0f;

    motion_setWheelPWM((int)left_pwm, (int)right_pwm);
}


/* Stop and distance commands */

/* Disable the active action and stop both motors. */
void motion_pidStop(void)
{
    pid_active = 0;
    target_speed = 0.0f;
    distance_mode = 0;
    turn_mode = 0;
    line_correction_pwm = 0;
    line_error_filtered = 0;
    motion_stop();
}

/* Start a forward movement for a specified distance */
void motion_pidMoveDistance(float distance_cm, float speed_cm_s)
{
    if (distance_cm <= 0.0f || speed_cm_s <= 0.0f) {
        motion_pidStop();
        motion_complete = 0;
        return;
    }

    motion_pidSetTargetSpeed(speed_cm_s);

    left_start_distance = encoder_getLeftDistance();
    right_start_distance = encoder_getRightDistance();

    target_distance = distance_cm;
    distance_mode = 1;
    motion_complete = 0;
}

/* Return the completion flag for the last distance or turn action. */
int motion_pidIsComplete(void)
{
    return motion_complete;
}


/* Encoder-based turning */

static void motion_pidStartTurn(float angle_degrees, int direction)
{
    if (angle_degrees <= 0.0f) {
        motion_pidStop();
        motion_complete = 0;
        return;
    }

    /* Disable forward PID and distance movement */
    motion_pidStop();
    motion_complete = 0;

    turn_left_start = encoder_getLeftDistance();
    turn_right_start = encoder_getRightDistance();

    /* Distance each wheel travels for an in-place turn */
    turn_target_cm =
        PI_VALUE * WHEEL_TRACK_CM * angle_degrees / 360.0f;

    turn_direction = direction;
    turn_boost_left = 0;
    turn_boost_right = 0;
    turn_check_left = 0.0f;
    turn_check_right = 0.0f;
    turn_check_s = 0.0f;
    turn_mode = 1;

    /* Opposite wheel directions */
    if (direction < 0) {
        motion_setWheelPWM(-TURN_PWM, TURN_PWM);
    } else {
        motion_setWheelPWM(TURN_PWM, -TURN_PWM);
    }
}

void motion_pidTurnLeft(float angle_degrees)
{
    motion_pidStartTurn(angle_degrees, -1);
}

void motion_pidTurnRight(float angle_degrees)
{
    motion_pidStartTurn(angle_degrees, 1);
}
