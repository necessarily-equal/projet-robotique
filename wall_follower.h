/**
 * @file maze_control.h
 * @brief 
 */

#ifndef _MAZE_CONTROL_H_
#define _MAZE_CONTROL_H_

#include <ch.h>

/*===========================================================================*/
/*  External declarations                                                    */
/*===========================================================================*/

void create_wall_follower_thd(void);
void stop_wall_follower_thd(void);
void pause_wall_follower_thd(void);
void resume_wall_follower_thd(void);
bool wall_follower_thd_status(void);

binary_semaphore_t *get_edge_detected_semaphore_ptr(void);

void move_to_next_wall(void);

#endif /* _MAZE_CONTROL_H_ */