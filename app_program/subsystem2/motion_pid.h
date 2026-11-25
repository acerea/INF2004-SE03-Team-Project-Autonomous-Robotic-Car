#ifndef MOTION_PID_H
#define MOTION_PID_H

/* Initialise both wheel PID controllers. */
void motion_pidInit(void);

/* Set target speed for both wheels (cm/s) */
void motion_pidSetTargetSpeed(float speed_cm_s);

/* Perform one PID control update (s) */
void motion_pidUpdate(float dt_seconds);

/* Stop PID-controlled movement */
void motion_pidStop(void);

/* Start moving forward for a specified distance (cm) */
void motion_pidMoveDistance(float distance_cm, float speed_cm_s);

/* Same, driving backwards (no line steering). */
void motion_pidMoveBackDistance(float distance_cm, float speed_cm_s);

/* Check whether requested distance or turn has completed (return 1)*/
int motion_pidIsComplete(void);

/* Turn robot by a specified angle (positive degrees) */
void motion_pidTurnLeft(float angle_degrees);
void motion_pidTurnRight(float angle_degrees);

/* Change forward speed while preserving an active distance target. */
void motion_pidChangeSpeed(float speed_cm_s);

/* Apply line-following correction in addition to encoder/PID correction.
 * Positive line_pos means the line is to the right. */
void motion_pidSetLineCorrection(int line_pos);

#endif /* MOTION_PID_H */
