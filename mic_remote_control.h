/**
 * @file    mic_processing.h
 * @brief   Exported functions for e-puck 2 audio control
 */

#ifndef _MOD_AUDIO_PROCESSING_H_
#define _MOD_AUDIO_PROCESSING_H_

typedef enum {
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

void create_mic_selector_thd(void);

void stop_mic_selector_thd(void);
void pause_mic_selector_thd(void);
void resume_mic_selector_thd(void);

#endif /* _MOD_AUDIO_PROCESSING_H_ */
