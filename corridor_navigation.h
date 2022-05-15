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

void create_corridor_navigation_thd(void);
void disable_corridor_navigation_thd(void);
void navigate_corridor(void);

bool corridor_navigation_thd_status(void);

binary_semaphore_t *get_corridor_end_detected_semaphore_ptr(void);

#endif /* _MAZE_CONTROL_H_ */
