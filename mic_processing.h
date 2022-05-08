/**
 * @file    mic_processing.h
 * @brief   Exported functions for e-puck 2 audio control
 */

#ifndef _MOD_AUDIO_PROCESSING_H_
#define _MOD_AUDIO_PROCESSING_H_

/*===========================================================================*/
/*  Module data structures and types                                         */
/*===========================================================================*/

typedef enum{
    IDLE_CMD = 0,       // put the e-puck in mic idle
    WAIT_CMD,           // prevents the e-puck from moving
    MOVE_CMD,           // allows the e-puck to move
    RST_MAZE_CMD,       // reset maze stored in memory
    START_MAZE_CMD,     // start the maze navigation
    U_TURN_CMD,         // order the e-puck to u turn (180deg)
    TURN_LEFT_CMD,      // order the e-puck to turn left
    TURN_RIGHT_CMD,     // order the e-puck to turn right
    SLOW_DOWN_CMD,      // order the e-puck to go in slow speed
    SPEED_UP_CMD,       // order the e-puck to go in "high" speed
} command_t;

/*========================================================================*/
/*  External declarations                                                 */
/*========================================================================*/

/**
 * @brief 				Init the microphones' thread
 */
void mic_processing_init(void);

#endif /* _MOD_AUDIO_PROCESSING_H_ */
