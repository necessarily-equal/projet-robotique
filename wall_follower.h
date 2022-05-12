/**
 * @file maze_control.h
 * @brief 
 */

#ifndef _MAZE_CONTROL_H_
#define _MAZE_CONTROL_H_

typedef struct junction_allowed_directions {
    bool left;
    bool straight;
    bool right;
    bool back;
} junction_allowd_directions_t ;

/*===========================================================================*/
/*  External declarations                                                    */
/*===========================================================================*/

void create_wall_follower_thd(void);
void stop_wall_follower_thd(void);
void pause_wall_follower_thd(void);
void resume_wall_follower_thd(void);

bool wall_follower_thd_state(void);

bool reached_end_of_wall(void);

void create_junction_study_thd(void);
void stop_junction_study_thd(void);
void pause_junction_study_thd(void);
void resume_junction_study_thd(void);

junction_allowd_directions_t get_directions(void);

#endif /* _MAZE_CONTROL_H_ */