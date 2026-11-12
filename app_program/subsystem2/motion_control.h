#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

/* Initialise both motor drivers. */
void motion_init(void);

/* Basic movement of the motors (speed: 0-100% PWM) */ 
void motion_moveForward(int speed);
void motion_moveBackward(int speed);
void motion_turnLeft(int speed);
void motion_turnRight(int speed);

/* Stop both motor outputs. */
void motion_stop(void);

/* Independent wheel PWM: -100 to +100.
 * Positive = forward, negative = backward, zero = stop.
 */
void motion_setWheelPWM(int left_pwm, int right_pwm);

#endif /* MOTION_CONTROL_H */