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
 * @brief 
 * 
 * @param direction 
 */
void right_angle_turn(rotation_t direction);

/**
 * @brief 
 * 
 */
void u_turn(void);

/**
 * @brief 
 * 
 * @param position 
 * @param direction 
 */
void turn(float position, rotation_t direction);

/**
 * @brief 
 * 
 * @param position 
 * @param sense 
 */
void move(float position, direction_t sense);

/**
 * @brief               Change the motor speed
 * 
 * @param speed         new motor speed in 
 */
void update_speed(uint16_t updated_speed);

/**
 * @brief Set the manual speed object
 * 
 * @param left_speed 
 * @param right_speed 
 */
void set_manual_speed(uint16_t left_speed, uint16_t right_speed);

/**
 * @brief Get the speed
 * 
 * @return uint16_t 
 */
uint16_t get_speed(void);

/**
 * @brief 
 * 
 */
void stop_move(void);

#endif /* _MOVE_COMMAND_H_ */