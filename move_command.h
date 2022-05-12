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

/**
 * @brief       Create the motor thread, this thread monitors the current
 *              current position of the motors with the target ones.
 * 
 */
void create_motor_thd(void);

/**
 * @brief       Terminate the motor thread
 * 
 */
void stop_motor_thd(void);

/**
 * @brief       Put the motor thread into sleep, motor are set to null speed.
 * 
 */
void pause_motor_thd(void);

/**
 * @brief       Wake up the motor thread from sleep
 * 
 */
void resume_motor_thd(void);

/**
 * @brief       return true if the motors have not reached the previous
 *              specified target position.
 * 
 * @return true     position not reached and motors running
 * @return false    motors in idle and position reached
 */
bool motor_is_moving(void);

void right_angle_turn(direction_t direction);
void u_turn(void);

void turn(float position, rotation_t direction);

void move(float position, direction_t direction);

/**
 * @brief Set the default speed (500 step/s)
 * 
 */
void set_default_speed(void);

void set_current_speed(int16_t new_speed);

int16_t get_current_speed(void);

/**
 * @brief manual override of the left/right speed for precise control
 * 
 * @param left_speed 
 * @param right_speed 
 */
void set_lr_speed(int left_speed, int right_speed);

void stop_move(void);

#endif /* _MOVE_COMMAND_H_ */