/**
 * @file maze_control.c
 * @brief
 */

// C standard headers
#include <stdio.h>
#include <string.h>

// ChibiOS headers
#include "hal.h"
#include "ch.h"
#include "chschd.h"

// e-puck 2 main processor headers
#include "sensors/proximity.h"

// Module headers
#include "maze_control.h"
#include <ir_sensors.h>
//#include <tof_sensor.h>
#include <move_command.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define ACTION_START        'E'     // begin maze navigation
#define ACTION_LEFT         'L'     // turn left
#define ACTION_STRAIGHT     'S'     // go straight forward
#define ACTION_RIGHT        'R'     // turn right
#define ACTION_BACK         'B'     // u trurn
#define ACTION_END          'D'     // detect the maze exit
#define ACTION_VOID         'V'     // no action found
#define ACTION_DELAY        2000

#define WALL_THRESHOLD      80
#define IR1                 0
#define IR2                 1
#define IR3                 2
#define IR4                 3
#define IR5                 4
#define IR6                 5
#define IR7                 6
#define IR8                 7

#define MAZE_HALF_WIDTH     1.5
#define CORRECTION_ANGLE    0.2

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

// thread constants
static bool is_paused = false;
static bool is_navigating_maze = false;
// maze constants
static bool exited_maze = false;
static bool optimized_maze = false;
static bool navigate_optimized_path = false;
static char saved_actions[100] = ""; // sizeof for number of actions

/*===========================================================================*/
/* Module thread pointers.                                                   */
/*===========================================================================*/

static thread_t *ptr_maze_thd;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

void navigate_to_next_junction(void)
{
    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus,"/proximity");
    proximity_msg_t prox_values = {0u};

    int16_t left_speed = 0, right_speed = 0;
    int16_t left_ir = 0, right_ir = 0;

    while (true) {
        //Update ir prox values
        messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

        //Navigation
        if(!is_paused){
            left_speed = get_speed();
            left_speed -= prox_values.delta[IR1]*2;
            left_speed -= prox_values.delta[IR2];

            right_speed = get_speed();
            right_speed -= prox_values.delta[IR8]*2;
            right_speed -= prox_values.delta[IR7];

            set_manual_speed(left_speed, right_speed);
        }

        chThdSleepMicroseconds(100);
        //Junction detection
        left_ir = prox_values.delta[IR6];
        right_ir = prox_values.delta[IR3];
        if (left_ir<WALL_THRESHOLD || right_ir<WALL_THRESHOLD || tof_wall_too_close()){
            break;
        }
    }
    stop_move();
}

char junction_action(void)
{
    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus,
        "/proximity");
    proximity_msg_t prox_values = {0u};

    messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

    int16_t left_ir = prox_values.delta[IR6];
    int16_t right_ir = prox_values.delta[IR3];

    //Simple LSRB navigation algorithm based on three detectors
    if (left_ir<WALL_THRESHOLD && tof_wall_too_close()
        && right_ir>WALL_THRESHOLD) //Left turn
        return ACTION_LEFT;
    else if (left_ir>WALL_THRESHOLD && tof_wall_too_close() &&
        right_ir<WALL_THRESHOLD) //Right Turn
        return ACTION_RIGHT;
    else if (left_ir<WALL_THRESHOLD && tof_wall_too_close() &&
        right_ir<WALL_THRESHOLD) //T Intersection
        return ACTION_LEFT; //As left is possible
    else if (left_ir<WALL_THRESHOLD && !tof_wall_too_close() &&
        right_ir>WALL_THRESHOLD) //Left T Intersection
        return ACTION_LEFT; // As Left is possible
    else if (left_ir>WALL_THRESHOLD && !tof_wall_too_close() &&
        right_ir<WALL_THRESHOLD) //Right T Tntersection
        return ACTION_STRAIGHT; //As Straight path is possible
    else if (left_ir>WALL_THRESHOLD && tof_wall_too_close() &&
        right_ir>WALL_THRESHOLD) //Dead End
        return ACTION_BACK;//As no other direction is possible
    else if (left_ir<WALL_THRESHOLD && !tof_wall_too_close() &&
        right_ir<WALL_THRESHOLD) //4 Lane intersection
        return ACTION_LEFT;// As Left is possible
    return ACTION_VOID;
}

void navigate_junction(char action){
    switch (action)
    {
    case ACTION_START:
        move(MAZE_HALF_WIDTH, FORWARD);
        exited_maze = false;
        break;
    case ACTION_LEFT:
        right_angle_turn(COUNTERCLOCKWISE);
        break;
    case ACTION_STRAIGHT:
        move(MAZE_HALF_WIDTH*2, FORWARD);
        break;
    case ACTION_RIGHT:
        right_angle_turn(CLOCKWISE);
        break;
    case ACTION_BACK:
        u_turn();
        break;
    case ACTION_END:
        stop_move();
        exited_maze = true;
        break;
    default:
        break;
    }
}

/*===========================================================================*/
/* Module threads                                                            */
/*===========================================================================*/

static THD_WORKING_AREA(wa_maze_thd, 1024);
static THD_FUNCTION(thd_maze, arg)
{
	chRegSetThreadName(__FUNCTION__);
    (void) arg;

    uint16_t i = 0;
    set_manual_speed(300, 300);
    chThdSleepMilliseconds(ACTION_DELAY);
    navigate_to_next_junction();

    while(!exited_maze && !chThdShouldTerminateX()){
        chSysLock();
        if(is_paused)
            chSchGoSleepS(CH_STATE_SUSPENDED);
        chSysUnlock();
        //if(navigate_optimized_path && optimized_maze){
            // to do !!!
        //}
        //else{
            //navigate_to_next_junction();
            //saved_actions[i] = junction_action();
            //navigate_junction(saved_actions[i]);
            //i++;
            //
        //}
        //navigate_junction(junction_action());
        chThdSleepMilliseconds(ACTION_DELAY);
    }

    //if(!optimized_maze && !chThdShouldTerminateX()){
        // optimizes maze path
    //}

    is_navigating_maze = false;
    chThdExit(0);
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void maze_reset(void)
{
    exited_maze = false;
    memset(saved_actions, 0, sizeof(saved_actions));
    optimized_maze = false;

    stop_move();
}

void maze_create_thd(void)
{
    if(!is_navigating_maze) {
        ptr_maze_thd = chThdCreateStatic(wa_maze_thd, sizeof(wa_maze_thd),
                                         NORMALPRIO, thd_maze, NULL);
        is_navigating_maze = true;
    }
}

void maze_stop_thd(void)
{
    if (is_navigating_maze) {
        maze_resume_thd();
        chThdTerminate(ptr_maze_thd);
        chThdWait(ptr_maze_thd);
        is_navigating_maze = false;
        is_paused = false;

        stop_move();
    }
}

void maze_pause_thd(void)
{
    if (is_navigating_maze){
        is_paused = true;
        stop_move();
    }
}

void maze_resume_thd(void)
{
    chSysLock();
    if (is_navigating_maze && is_paused){
        chSchWakeupS(ptr_maze_thd, CH_STATE_READY);
        is_paused = false;
    }
    chSysUnlock();
}

bool maze_get_state(void)
{
    return is_navigating_maze;
}

void select_optimized_path(void)
{
    if(!is_navigating_maze && exited_maze){
        navigate_optimized_path = true;
    }
}
