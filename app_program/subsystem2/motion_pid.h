#ifndef MOTION_PID_H
#define MOTION_PID_H

/* Initialise PID controllers for both wheels */
void motion_pidInit(void);

/* Set target speed for both wheels (cm/s) */
void motion_pidSetTargetSpeed(float speed_cm_s);

/* Perform one PID control update */
void motion_pidUpdate(float dt_seconds);

/* Stop PID-controlled movement */
void motion_pidStop(void);

#endif /* MOTION_PID_H */