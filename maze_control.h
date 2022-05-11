/**
 * @file maze_control.h
 * @brief 
 */

#ifndef _MAZE_CONTROL_H_
#define _MAZE_CONTROL_H_

/*===========================================================================*/
/*  External declarations                                                    */
/*===========================================================================*/

void create_maze_navigation_thd(void);

void stop_maze_navigation_thd(void);

void pause_maze_navigation_thd(void);

void resume_maze_navigation_thd(void);

bool maze_get_state(void);

void reset_maze(void);

void select_optimized_path(void);

void create_link_navigation_thd(void);

void stop_link_navigation_thd(void);

void pause_link_navigation_thd(void);

void resume_link_navigation_thd(void);

bool link_get_state(void);

#endif /* _MAZE_CONTROL_H_ */