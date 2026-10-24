/* Import the necessary headers */
#include "motion_control.h"
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include <sys/sysdef.h>

/* Define Cytron Robo Pico motor pins */
#define M1A 8
#define M1B 9
#define M2A 10
#define M2B 11

/* PWM slices 4 and 5 clock dividers */
#define PWM_DIV_REG(slice) \
    (*(volatile unsigned int *)(0x40050004u + (slice) * 0x14u))

/* Initialise motor PWM */
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

    for (int i = 0; i < 4; i++) {
        pwm_set_cc(pins[i], 0);
        pwm_set_wrap(pins[i], 999);
        pwm_set_pin(pins[i]);
        pwm_set_enabled(pins[i], TRUE);
    }
}

// Set PWM duty cycle: 0-100%
static void motor_pwm(UINT pin, int speed)
{
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;

    pwm_set_cc(pin, (UW)speed * 10u);
}

/* Left Motor: M1B forward, M1A backward */
static void left_motor(int speed)
{
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;

    if (speed > 0) {
        motor_pwm(M1A, 0);
        motor_pwm(M1B, speed);
    } else {
        motor_pwm(M1B, 0);
        motor_pwm(M1A, -speed);
    }
}

/* Right motor: M2A forward, M2B backward */
static void right_motor(int speed)
{
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;

    if (speed > 0) {
        motor_pwm(M2B, 0);
        motor_pwm(M2A, speed);
    } else {
        motor_pwm(M2A, 0);
        motor_pwm(M2B, -speed);
    }
}

/* Public movement APIs */
void motion_moveForward(int speed)
{
    left_motor(speed);
    right_motor(speed);
}

void motion_moveBackward(int speed)
{
    left_motor(-speed);
    right_motor(-speed);
}

void motion_turnLeft(int speed)
{
    left_motor(-speed);
    right_motor(speed);
}

void motion_turnRight(int speed)
{
    left_motor(speed);
    right_motor(-speed);
}

void motion_stop(void)
{
    left_motor(0);
    right_motor(0);
}
