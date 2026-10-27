#include "motion_pid.h"
#include "pid_control.h"
#include "wheel_encoder.h"
#include "motion_control.h"

/* PID controllers for left and right wheels */
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

/* FR5: Preliminary straight-line correction gain */
#define STRAIGHT_KP 0.1f

/* FR6: Encoder-based turning */
#define WHEEL_TRACK_CM 9.9f
#define PI_VALUE 3.14159265f
#define TURN_PWM 25

static int turn_mode = 0;
static int turn_direction = 0;  /* -1 = left, +1 = right */
static float turn_target_cm = 0.0f;
static float turn_left_start = 0.0f;
static float turn_right_start = 0.0f;

/* Initialise both PID controllers */
void motion_pidInit(void)
{
    /* Preliminary gains from your PID tests */
    speed_pid_init(&left_pid, 2.0f, 1.0f, 0.01f, 30.0f);
    speed_pid_init(&right_pid, 2.0f, 1.0f, 0.01f, 30.0f);

    left_previous_count = encoder_getLeftCount();
    right_previous_count = encoder_getRightCount();

    target_speed = 0.0f;
    pid_active = 0;
    distance_mode = 0;
    motion_complete = 0;
    turn_mode = 0;
}

/* Set the target speed for forward movement */
void motion_pidSetTargetSpeed(float speed_cm_s)
{
    if (speed_cm_s <= 0.0f) {
        motion_pidStop();
        return;
    }

    /* Reset PID state for a new movement */
    speed_pid_init(&left_pid, 2.0f, 1.0f, 0.01f, 30.0f);
    speed_pid_init(&right_pid, 2.0f, 1.0f, 0.01f, 30.0f);

    left_previous_count = encoder_getLeftCount();
    right_previous_count = encoder_getRightCount();

    target_speed = speed_cm_s;
    pid_active = 1;
    distance_mode = 0;
    motion_complete = 0;
}

/* Run one PID control iteration */
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

        /* Stop each wheel independently when its target is reached */
        int left_pwm = (left_travelled >= turn_target_cm)
               ? 0 : turn_direction * TURN_PWM;

        int right_pwm = (right_travelled >= turn_target_cm)
               ? 0 : -turn_direction * TURN_PWM;

        motion_setWheelPWM(left_pwm, right_pwm);

        /* Complete only when both wheels reach their targets */
        if (left_travelled >= turn_target_cm &&
            right_travelled >= turn_target_cm) {
            motion_pidStop();
            motion_complete = 1;
        }

        return;
    }

    if (!pid_active || dt_seconds <= 0.0f) {
        return;
    }

    if (distance_mode) {
        float left_travelled =
            encoder_getLeftDistance() - left_start_distance;

        float right_travelled =
            encoder_getRightDistance() - right_start_distance;

        float travelled =
            (left_travelled + right_travelled) / 2.0f;

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
    float correction = STRAIGHT_KP * speed_error;

    /* Slow the faster wheel and boost the slower wheel */
    left_pwm -= correction;
    right_pwm += correction;

    /* Limit PWM to forward range */
    if (left_pwm < 0.0f) left_pwm = 0.0f;
    if (left_pwm > 100.0f) left_pwm = 100.0f;

    if (right_pwm < 0.0f) right_pwm = 0.0f;
    if (right_pwm > 100.0f) right_pwm = 100.0f;

    motion_setWheelPWM((int)left_pwm, (int)right_pwm);
}

/* Stop PID-controlled movement */
void motion_pidStop(void)
{
    pid_active = 0;
    target_speed = 0.0f;
    distance_mode = 0;
    turn_mode = 0;
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

/* Check whether the requested distance was reached */
int motion_pidIsComplete(void)
{
    return motion_complete;
}

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