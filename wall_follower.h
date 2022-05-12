/**
 * @file maze_control.h
 * @brief 
 */

#ifndef _MAZE_CONTROL_H_
#define _MAZE_CONTROL_H_

/*===========================================================================*/
/*  External declarations                                                    */
/*===========================================================================*/

void create_wall_follower_thd(void);

void stop_wall_follower_thd(void);

void pause_wall_follower_thd(void);

void resume_wall_follower_thd(void);

bool wall_follower_thd_state(void);

bool reached_end_of_wall(void);

#endif /* _MAZE_CONTROL_H_ */