#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

/* Initialise both motor 1 and motor 2 */
void motion_init(void);

/* Basic movement of the motors (speed: 0-100% PWM) */ 
void motion_moveForward(int speed);
void motion_moveBackward(int speed);
void motion_turnLeft(int speed);
void motion_turnRight(int speed);
void motion_stop(void);

/* Set left and right wheel PWM independently */
void motion_setWheelPWM(int left_pwm, int right_pwm);

#endif /* MOTION_CONTROL_H */