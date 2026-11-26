#ifndef MOTION_PID_H
#define MOTION_PID_H

/* Initialise both wheel PID controllers. */
void motion_pid_init(void);

/* Set target speed for both wheels (cm/s) */
void motion_pid_set_target_speed(float speed_cm_s);

/* Perform one PID control update (s) */
void motion_pid_update(float dt_seconds);

/* Stop PID-controlled movement */
void motion_pid_stop(void);

/* Start moving forward for a specified distance (cm) */
void motion_pid_move_distance(float distance_cm, float speed_cm_s);

/* Same, driving backwards (no line steering). */
void motion_pid_move_back_distance(float distance_cm, float speed_cm_s);

/* Check whether requested distance or turn has completed (return 1)*/
int motion_pid_is_complete(void);

/* Turn robot by a specified angle (positive degrees) */
void motion_pid_turn_left(float angle_degrees);
void motion_pid_turn_right(float angle_degrees);

/* Change forward speed while preserving an active distance target. */
void motion_pid_change_speed(float speed_cm_s);

/* Apply line-following correction in addition to encoder/PID correction.
 * Positive line_pos means the line is to the right. */
void motion_pid_set_line_correction(int line_pos);

#endif /* MOTION_PID_H */
