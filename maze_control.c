/**
 * @file maze_control.c
 * @brief
 */

// C standard headers
#include <stdio.h>
#include <string.h>
#include <math.h>

// ChibiOS headers
#include "hal.h"
#include "ch.h"
#include "chschd.h"

// e-puck 2 main processor headers
#include "sensors/proximity.h"

#include <chprintf.h>

// Module headers
#include "maze_control.h"
#include <ir_sensors.h>
#include <move_command.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define ACTION_START            'E'     // begin maze navigation
#define ACTION_LEFT             'L'     // turn left
#define ACTION_STRAIGHT         'S'     // go straight forward
#define ACTION_RIGHT            'R'     // turn right
#define ACTION_BACK             'B'     // u trurn
#define ACTION_END              'D'     // detect the maze exit
#define ACTION_VOID             'V'     // no action found
#define ACTION_DELAY            2000



#define JUNCTION_THRESHOLD      100


#define MAZE_NAVIGATION_PERIOD  100
#define LINK_NAVIGATION_PERIOD  100

/*===========================================================================*/
/* Wall PI constants.                                                        */
/*===========================================================================*/

#define WALL_TARGET             250
#define CORRECTION_THRESHOLD    5

#define LINK_ERROR_THRESHOLD    0.1f
#define LINK_UPPER_CLAMP        100
#define LINK_LOWER_CLAMP        -100
#define LINK_KP                 0.25f
#define LINK_KI                 0.5f
#define LINK_KD                 5.0f
#define MAX_SUM_ERROR 			100

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

// thread constants
static bool maze_navigation_paused = false;
static bool is_navigating_maze = false;

static bool link_navigation_paused = false;
static bool is_navigating_link = false;

// maze constants
static bool exited_maze = false;
static bool optimized_maze = false;
static bool navigate_optimized_path = false;
static bool junction_detected = false;
static char saved_actions[100] = "";

/*===========================================================================*/
/* Module thread pointers.                                                   */
/*===========================================================================*/

static thread_t *ptr_maze_navigation_thd;
static thread_t *ptr_link_navigation_thd;

int16_t pid_regulator(float current, float target)
{
    float error = 0.0f;
    float control = 0.0f;
    float derivative = 0.0f;

    //Static for integral and derivative
    static float sum_error = 0.0f;
    static float last_error = 0.0f;

    //Calculate the error
    error = current - target;

    if(fabs(error) < LINK_ERROR_THRESHOLD)
        return 0;

    //Calculate the integral
    sum_error += error;

    //Calcualte the derivative
    derivative = error - last_error;

    if(sum_error > MAX_SUM_ERROR)
		sum_error = MAX_SUM_ERROR;
	else if(sum_error < -MAX_SUM_ERROR)
		sum_error = -MAX_SUM_ERROR;

    last_error = error;
    control = LINK_KP * error + LINK_KI * sum_error + LINK_KD * derivative;

    //output clamping
    if(control > LINK_UPPER_CLAMP)
		control = LINK_UPPER_CLAMP;
	else if(control < LINK_LOWER_CLAMP)
		control = LINK_LOWER_CLAMP;

    return (int16_t)control;
}

bool detect_junction(void)
{
    return false;
}

/*===========================================================================*/
/* Module threads.                                                           */
/*===========================================================================*/

static THD_WORKING_AREA(wa_maze_navigation_thd, 1024);
static THD_FUNCTION(maze_navigation_thd, arg)
{
    chRegSetThreadName(__FUNCTION__);
	(void)arg;

    while(!chThdShouldTerminateX() && !exited_maze){
        //-> takes action
        //-> stores actions
        //-> launch segment navigation thd
        //-> scan for junction
        //-> if found 
        //-> stop segment navigation thd
        //repeat
        chThdSleepMilliseconds(MAZE_NAVIGATION_PERIOD);
    }

    is_navigating_maze = false;
    chThdExit(0);
}

void create_maze_navigation_thd(void)
{
    if(!is_navigating_maze)
    {
        ptr_maze_navigation_thd = chThdCreateStatic(wa_maze_navigation_thd,
            sizeof(wa_maze_navigation_thd), NORMALPRIO, maze_navigation_thd,
            NULL);
        is_navigating_maze = true;
    }
}

void stop_maze_navigation_thd(void)
{
    if(is_navigating_maze)
    {
        resume_maze_navigation_thd();
        chThdTerminate(ptr_maze_navigation_thd);
        chThdWait(ptr_maze_navigation_thd);
        maze_navigation_paused = false;
        is_navigating_maze = false;
    }
}

void pause_maze_navigation_thd(void)
{
    if(is_navigating_maze)
        maze_navigation_paused = true;
}

void resume_maze_navigation_thd(void)
{
    chSysLock();
    if(maze_navigation_paused && is_navigating_maze){
        chSchWakeupS(ptr_maze_navigation_thd, CH_STATE_READY);
        maze_navigation_paused = false;
    }
    chSysUnlock();
}

bool maze_get_state(void)
{
    return is_navigating_maze;
}

void reset_maze(void)
{
    exited_maze = false;
    optimized_maze = false;
    navigate_optimized_path = false;
    memset(saved_actions, 0, sizeof(saved_actions));
    pause_maze_navigation_thd();
    pause_link_navigation_thd();
    stop_move();
}

void select_optimized_path(void)
{
    if(!is_navigating_maze && exited_maze){
        navigate_optimized_path = true;
    }
}

static THD_WORKING_AREA(wa_link_navigation_thd, 1024);
static THD_FUNCTION(link_navigation_thd, arg)
{
    chRegSetThreadName(__FUNCTION__);
	(void)arg;

    systime_t time;
    int delta_speed = 0;

    while(!chThdShouldTerminateX())
    {
        //Thread Sleep
        chSysLock();
        if(link_navigation_paused)
            chSchGoSleepS(CH_STATE_SUSPENDED);
        chSysUnlock();
        //Thread loop function
        time = chVTGetSystemTime();

        delta_speed = pid_regulator(get_ir_delta(IR6), WALL_TARGET);
        if(fabs(delta_speed) < CORRECTION_THRESHOLD)
            delta_speed = 0;
        set_lr_speed(get_current_speed() + delta_speed,
                     get_current_speed() - delta_speed);
        //Thread exit condition
        if(get_ir_delta(IR6) < JUNCTION_THRESHOLD){
            junction_detected = true;
            break;
        }

        //Thread refresh rate
        chThdSleepUntilWindowed(time, time + MS2ST(LINK_NAVIGATION_PERIOD));
    }

    is_navigating_link = false;
    stop_move();
    chThdExit(0);
}

void create_link_navigation_thd(void)
{
    if(!is_navigating_link)
    {
        ptr_link_navigation_thd = chThdCreateStatic(wa_link_navigation_thd,
            sizeof(wa_link_navigation_thd), NORMALPRIO, link_navigation_thd,
            NULL);
        is_navigating_link = true;
    }
}

void stop_link_navigation_thd(void)
{
    if(is_navigating_link)
    {
        resume_link_navigation_thd();
        chThdTerminate(ptr_link_navigation_thd);
        chThdWait(ptr_link_navigation_thd);
        stop_move();
        link_navigation_paused = false;
        is_navigating_link = false;
    }
}

void pause_link_navigation_thd(void)
{
    if(is_navigating_link)
        link_navigation_paused = true;
}

void resume_link_navigation_thd(void)
{
    chSysLock();
    if(link_navigation_paused && is_navigating_link){
        chSchWakeupS(ptr_link_navigation_thd, CH_STATE_READY);
        link_navigation_paused = false;
    }
    chSysUnlock();
}

bool link_get_state(void)
{
    return is_navigating_link;
}
