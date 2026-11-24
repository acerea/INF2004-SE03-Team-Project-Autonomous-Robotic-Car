#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include <sys/sysdef.h>

#include "motion_control.h"

/* Cytron Robo Pico motor pins */
#define M1A 8
#define M1B 9
#define M2A 10
#define M2B 11

/* M1 uses PWM slice 4, M2 uses PWM slice 5. */
/* Last signed PWM set on each wheel, for telemetry only. */
static volatile int last_left_pwm = 0;
static volatile int last_right_pwm = 0;

#define PWM_DIV_REG(slice) \
    (*(volatile unsigned int *)(0x40050004u + (slice) * 0x14u))

/* Motor PWM Initialisation */
void motion_init(void)
{
    const UINT pins[] = {M1A, M1B, M2A, M2B};

    /* Release PWM peripheral from reset */
    clr_w(RESETS_RESET, RESETS_RESET_PWM);

    /* Wait until PWM hardware is ready */
    while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0u) {
    }

    /* Configure the clock divider for both motor PWM slices. */
    PWM_DIV_REG(4u) = 125u << 4;
    PWM_DIV_REG(5u) = 125u << 4;

    for (int i = 0; i < 4; i++) {
        pwm_set_cc(pins[i], 0);
        pwm_set_wrap(pins[i], 999);
        pwm_set_pin(pins[i]);
        pwm_set_enabled(pins[i], TRUE);
    }
}

/* Private motor helpers */

/* Set PWM duty cycle (0-100%) */
static void motor_pwm(UINT pin, int speed)
{
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;

    pwm_set_cc(pin, (UW)speed * 10u);
}

/* Left motor: positive drives M1B; negative drives M1A. */
static void motor_leftm1(int speed)
{
    if (speed > 100) {
        speed = 100;
    }

    if (speed < -100) {
        speed = -100;
    }

    last_left_pwm = speed;

    if (speed > 0) {
        motor_pwm(M1A, 0);
        motor_pwm(M1B, speed);
    } else {
        motor_pwm(M1B, 0);
        motor_pwm(M1A, -speed);
    }
}

/* Right motor: positive drives M2A; negative drives M2B. */
static void motor_rightm2(int speed)
{
    if (speed > 100) {
        speed = 100;
    }

    if (speed < -100) {
        speed = -100;
    }

    last_right_pwm = speed;

    if (speed > 0) {
        motor_pwm(M2B, 0);
        motor_pwm(M2A, speed);
    } else {
        motor_pwm(M2A, 0);
        motor_pwm(M2B, -speed);
    }
}

/* Soft start: a wheel's PWM may rise by at most MOTION_RAMP_STEP per call
 * (called every 20 ms), so 0 -> 70 % takes about 0.3 s. This limits the
 * current spike when the motors start, which otherwise browns out the Pico
 * on a 5 V 2 A power bank. Slowing down and stopping take effect at once.
 */
#define MOTION_RAMP_STEP 5

static int motion_ramp(int target, int current)
{
    int magnitude;

    if (target == 0) {
        return 0;
    }

    /* Reversing starts again from zero. */
    if ((target > 0 && current < 0) || (target < 0 && current > 0)) {
        current = 0;
    }

    magnitude = (current < 0) ? -current : current;

    if (((target < 0) ? -target : target) <= magnitude) {
        return target;
    }

    magnitude += MOTION_RAMP_STEP;

    if (target > 0) {
        return (magnitude < target) ? magnitude : target;
    }

    return (magnitude < -target) ? -magnitude : target;
}

/* Public movement commands */

/* Set signed wheel PWM independently: -100 to +100. */
void motion_setWheelPWM(int left_pwm, int right_pwm)
{
    motor_leftm1(motion_ramp(left_pwm, last_left_pwm));
    motor_rightm2(motion_ramp(right_pwm, last_right_pwm));
}

/* Move both wheels forward */
void motion_moveForward(int speed)
{
    motor_leftm1(speed);
    motor_rightm2(speed);
}

/* Move both wheels backward */
void motion_moveBackward(int speed)
{
    motor_leftm1(-speed);
    motor_rightm2(-speed);
}

/* Turn left by rotating wheels in opposite directions */
void motion_turnLeft(int speed)
{
    motor_leftm1(-speed);
    motor_rightm2(speed);
}

/* Turn right by rotating wheels in opposite directions */
void motion_turnRight(int speed)
{
    motor_leftm1(speed);
    motor_rightm2(-speed);
}

/* Read the last PWM set on each wheel (-100 to +100). */
void motion_getWheelPWM(int *left_pwm, int *right_pwm)
{
    *left_pwm = last_left_pwm;
    *right_pwm = last_right_pwm;
}

/* Stop both motors */
void motion_stop(void)
{
    motor_leftm1(0);
    motor_rightm2(0);
}
