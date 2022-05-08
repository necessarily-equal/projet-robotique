/**
 * @file maze_control.h
 * @brief 
 */

#ifndef _MAZE_CONTROL_H_
#define _MAZE_CONTROL_H_

/*===========================================================================*/
/*  External declarations                                                    */
/*===========================================================================*/

/**
 * @brief 
 * 
 */
void maze_reset(void);

/**
 * @brief 
 * 
 */
void maze_create_thd(void);

/**
 * @brief 
 * 
 */
void maze_stop_thd(void);

/**
 * @brief 
 * 
 */
void maze_pause_thd(void);

/**
 * @brief 
 * 
 */
void maze_resume_thd(void);

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool maze_get_state(void);

/**
 * @brief 
 * 
 */
void select_optimized_path(void);

#endif /* _MAZE_CONTROL_H_ */