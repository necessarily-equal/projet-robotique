/**
 * @file move_command.h
 * @brief 
 */


#ifndef _MOVE_COMMAND_H_
#define _MOVE_COMMAND_H_

/*===========================================================================*/
/*  Module data structures and types                                         */
/*===========================================================================*/

typedef enum {
    CLOCKWISE           = 1,
    COUNTERCLOCKWISE    = -1,
} rotation_t;

typedef enum {
    FORWARD     = 1,
    BACKWARD    = -1,
} direction_t;

/*===========================================================================*/
/*  External declarations                                                    */
/*===========================================================================*/

void create_motor_thd(void);
void stop_motor_thd(void);
void pause_motor_thd(void);
void resume_motor_thd(void);

bool motor_is_moving(void);

void right_angle_turn(direction_t direction);
void u_turn(void);
void turn(float position, rotation_t direction);

void move(float position, direction_t direction);

void set_default_speed(void);
void set_current_speed(int16_t new_speed);
int16_t get_current_speed(void);

void set_lr_speed(int left_speed, int right_speed);

void stop_move(void);

#endif /* _MOVE_COMMAND_H_ */