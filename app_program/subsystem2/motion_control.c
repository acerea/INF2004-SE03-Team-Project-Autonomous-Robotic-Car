#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include <sys/sysdef.h>

#include "motion_control.h"

/* --------------------------------
 * 1. Motor GPIO Configuration
 * -------------------------------- */

/* Cytron Robo Pico motor pins */
#define M1A 8
#define M1B 9
#define M2A 10
#define M2B 11

/* M1 uses PWM slice 4, M2 uses PWM slice 5. */
#define PWM_DIV_REG(slice) \
    (*(volatile unsigned int *)(0x40050004u + (slice) * 0x14u))

/* --------------------------------
 * 2. Motor PWM Initialisation
 * -------------------------------- */
void motion_init(void)
{
    const UINT pins[] = {M1A, M1B, M2A, M2B};

    /* Release PWM peripheral from reset */
    clr_w(RESETS_RESET, RESETS_RESET_PWM);

    /* Wait until PWM hardware is ready */
    while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0u) {
    }

    /* Set PWM clock divider to 125 */
    PWM_DIV_REG(4u) = 125u << 4;
    PWM_DIV_REG(5u) = 125u << 4;

    /* Set motor pins for PWM output */
    for (int i = 0; i < 4; i++) {
        pwm_set_cc(pins[i], 0);
        pwm_set_wrap(pins[i], 999);
        pwm_set_pin(pins[i]);
        pwm_set_enabled(pins[i], TRUE);
    }
}

/* --------------------------------
 * 3. PWM Duty Cycle Control
 * -------------------------------- */

 /* Set PWM duty cycle (0-100%) */
static void motor_pwm(UINT pin, int speed)
{
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;

    pwm_set_cc(pin, (UW)speed * 10u);
}

/* --------------------------------
 * 4. Motor Control Functions
 * -------------------------------- */

/* 
 * Left Motor (M1) 
 * Positive speed: Forward (M1B) 
 * Negative speed: Backward (M1A) 
*/
static void motor_leftm1(int speed)
{
    if (speed > 100) {
        speed = 100;
    }

    if (speed < -100) {
        speed = -100;
    }

    if (speed > 0) {
        motor_pwm(M1A, 0);
        motor_pwm(M1B, speed);
    } else {
        motor_pwm(M1B, 0);
        motor_pwm(M1A, -speed);
    }
}

/* 
 * Right Motor (M2) 
 * Positive speed: Forward (M2A) 
 * Negative speed: Backward (M2B) 
 */
static void motor_rightm2(int speed)
{
    if (speed > 100) {
        speed = 100;
    }

    if (speed < -100) {
        speed = -100;
    }

    if (speed > 0) {
        motor_pwm(M2B, 0);
        motor_pwm(M2A, speed);
    } else {
        motor_pwm(M2A, 0);
        motor_pwm(M2B, -speed);
    }
}

/* --------------------------------
 * 5. Public Motor Control APIs
 * -------------------------------- */
 
 /* Set left and right wheel PWM independently. */
void motion_setWheelPWM(int left_pwm, int right_pwm)
{
    motor_leftm1(left_pwm);
    motor_rightm2(right_pwm);
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

/* Stop both motors */
void motion_stop(void)
{
    motor_leftm1(0);
    motor_rightm2(0);
}
