/**
 * @file wall_follower.c
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
#include "leds.h"

// Module headers
#include "wall_follower.h"
#include <ir_sensors.h>
#include <move_command.h>

/*===========================================================================*/
/* Wall PID constants.                                                       */
/*===========================================================================*/

#define WALL_TARGET             250
#define CORRECTION_THRESHOLD    5

#define WALL_DISTANCE           110

#define LINK_ERROR_THRESHOLD    0.1f
#define LINK_UPPER_CLAMP        100
#define LINK_LOWER_CLAMP        -100
#define LINK_KP                 0.25f
#define LINK_KI                 0.5f
#define LINK_KD                 5.0f
#define MAX_SUM_ERROR 			100

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define SIDE_JUNCTION_THLD      100
#define FRONT_WALL_THLD         200

#define LINK_NAVIGATION_PERIOD  100
#define JUNCTION_STUDY_PERIOD   100

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static bool end_of_wall_detected = false;

//Threads variables
static bool wall_follower_paused = false;   //thread is paused
static bool wall_following_thd_active = false;      //thread is created

static bool junction_study_paused = false;
static bool junction_study_thd_active = false;

/*===========================================================================*/
/* Semaphores.                                                               */
/*===========================================================================*/

static BSEMAPHORE_DECL(junction_detected, TRUE);

/*===========================================================================*/
/* Module thread pointers.                                                   */
/*===========================================================================*/

static thread_t *ptr_wall_follower_thd;
static thread_t *ptr_study_junction_thd;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

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

bool detect_end_of_wall(void)
{
    if(get_ir_delta(IR6) < SIDE_JUNCTION_THLD || get_ir_delta(IR3) < SIDE_JUNCTION_THLD)
    {
        return true;
    }
    return false;
}

bool detect_dead_end(void)
{
    if(get_ir_delta(IR1) > FRONT_WALL_THLD || get_ir_delta(IR8) > FRONT_WALL_THLD){
        return true;
    }
    return false;
}

void move_to_junction_center(void)
{
    move(WALL_DISTANCE/20, FORWARD);
}

/*===========================================================================*/
/* Module threads.                                                           */
/*===========================================================================*/

static THD_WORKING_AREA(wa_wall_follower_thd, 1024);
static THD_FUNCTION(wall_follower_thd, arg)
{
    chRegSetThreadName(__FUNCTION__);
	(void)arg;

    systime_t time;
    int delta_speed = 0;

    while(!chThdShouldTerminateX())
    {
        //Thread Sleep
        chSysLock();
        if(wall_follower_paused)
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
        if(detect_end_of_wall() || detect_dead_end()){
            wall_follower_paused = true;
            chBSemSignal(&junction_detected);
            stop_move();
        }

        //Thread refresh rate
        chThdSleepUntilWindowed(time, time + MS2ST(LINK_NAVIGATION_PERIOD));
    }

    wall_following_thd_active = false;
    chThdExit(0);
}

static THD_WORKING_AREA(wa_junction_study_thd, 1024);
static THD_FUNCTION(junction_study_thd, arg)
{
    chRegSetThreadName(__FUNCTION__);
	(void)arg;

    systime_t time;

    while(!chThdShouldTerminateX())
    {
        //Thread Sleep
        chSysLock();
        if(junction_study_paused)
            chSchGoSleepS(CH_STATE_SUSPENDED);
        chSysUnlock();
        //Thread body
        time = chVTGetSystemTime();
        chBSemWait(&junction_detected);
        set_body_led(true);
        move_to_junction_center();
        set_body_led(false);
        //update_junction_table();
        //do actions
        //Thread refresh rate
        chThdSleepUntilWindowed(time, time + MS2ST(JUNCTION_STUDY_PERIOD));
    }   

    junction_study_thd_active = false;
    chThdExit(0); 
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void create_wall_follower_thd(void)
{
    if(!wall_following_thd_active)
    {
        ptr_wall_follower_thd = chThdCreateStatic(wa_wall_follower_thd,
            sizeof(wa_wall_follower_thd), NORMALPRIO, wall_follower_thd,
            NULL);
        wall_following_thd_active = true;
    }
}

void stop_wall_follower_thd(void)
{
    if(wall_following_thd_active)
    {
        resume_wall_follower_thd();
        chThdTerminate(ptr_wall_follower_thd);
        chThdWait(ptr_wall_follower_thd);
        stop_move();
        wall_follower_paused = false;
        wall_following_thd_active = false;
    }
}

void pause_wall_follower_thd(void)
{
    if(wall_following_thd_active)
        wall_follower_paused = true;
}

void resume_wall_follower_thd(void)
{
    chSysLock();
    if(wall_follower_paused && wall_following_thd_active){
        chSchWakeupS(ptr_wall_follower_thd, CH_STATE_READY);
        wall_follower_paused = false;
    }
    chSysUnlock();
}

///////////////////////////////////////////////////////////////////////

bool reached_end_of_wall(void)
{
    return (end_of_wall_detected && wall_follower_paused);
}

bool wall_follower_thd_state(void)
{
    return wall_following_thd_active;
}

//////////////////////////////////////////////////////////////////////

void create_junction_study_thd(void)
{
    if(!junction_study_thd_active)
    {
        ptr_study_junction_thd = chThdCreateStatic(wa_junction_study_thd,
            sizeof(wa_junction_study_thd), NORMALPRIO, junction_study_thd,
            NULL);
        junction_study_thd_active = true;
    }
}

void stop_junction_study_thd(void)
{
    if(junction_study_thd_active)
    {
        resume_junction_study_thd();
        chThdTerminate(ptr_study_junction_thd);
        chThdWait(ptr_study_junction_thd);
        stop_move();
        junction_study_paused = false;
        junction_study_thd_active = false;
    }
}

void pause_junction_study_thd(void)
{
    if(junction_study_thd_active)
        junction_study_paused = true;
}

void resume_junction_study_thd(void)
{
    chSysLock();
    if(junction_study_paused && junction_study_thd_active){
        chSchWakeupS(ptr_study_junction_thd, CH_STATE_READY);
        junction_study_paused = false;
    }
    chSysUnlock();
}