/**
 * @file wall_follower.c
 * @brief 
 * 
 */

// C standard headers
#include <math.h>

// ChibiOS headers
#include "hal.h"
#include "ch.h"
#include "chschd.h"

// e-puck 2 main processor headers
#include "sensors/proximity.h"
#include <chprintf.h>

// Module headers
#include "wall_follower.h"
#include <ir_sensors.h>
#include <move_command.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/
//PID constants.
#define WALL_TARGET             1000
#define CORRECTION_THRESHOLD    1

#define LINK_ERROR_THRESHOLD    0.1f
#define LINK_UPPER_CLAMP        100
#define LINK_LOWER_CLAMP        -100
#define LINK_KP                 0.01f
#define LINK_KI                 0.1f
#define LINK_KD                 5.0f
#define MAX_SUM_ERROR 			100
//Walls constants.
#define WALL_EDGE_THLD          150 // IR3 & IR6
#define FRONT_WALL_THLD         1500 // IR1 & IR8
#define FRONT_SIDE_WALL_THLD    1000 // IR2 & IR7

//Thread constants.
#define WALL_FOLLOWER_PERIOD    100

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static bool wall_follower_thd_created = false;
static bool wall_follower_thd_paused = false;

static bool left_wall_detected = false;
static bool right_wall_detected = false;
static bool front_wall_detected = false;

/*===========================================================================*/
/* Semaphores.                                                               */
/*===========================================================================*/

static BSEMAPHORE_DECL(wall_edge_detected, TRUE);

/*===========================================================================*/
/* Module thread pointers.                                                   */
/*===========================================================================*/

static thread_t *ptr_wall_follower_thd;

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

bool wall_detected(void) {
    return (front_wall_detected || right_wall_detected || left_wall_detected);
}

bool check_front_sensors(void) {
    //Front checks
    if(get_ir_delta(IR1) > FRONT_WALL_THLD) return true;
    if(get_ir_delta(IR2) > FRONT_SIDE_WALL_THLD) return true;
    //Front left and right checks
    if(get_ir_delta(IR8) > FRONT_WALL_THLD) return true;
    if(get_ir_delta(IR7) > FRONT_SIDE_WALL_THLD) return true;
    return false;
}

bool check_right_sensors(void) {
    if(get_ir_delta(IR2) > FRONT_SIDE_WALL_THLD) return true;
    if(get_ir_delta(IR3) > FRONT_SIDE_WALL_THLD) return true;
    return false;
}

bool check_left_sensors(void) {
    if(get_ir_delta(IR6) > FRONT_SIDE_WALL_THLD) return true;
    if(get_ir_delta(IR7) > FRONT_SIDE_WALL_THLD) return true;
    return false;
}

bool check_wall_edge(void) {
    if(get_ir_delta(IR3) < WALL_EDGE_THLD) return true;
    if(get_ir_delta(IR6) < WALL_EDGE_THLD) return true;
    return false;
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
        if(wall_follower_thd_paused)
            chSchGoSleepS(CH_STATE_SUSPENDED);
        chSysUnlock();
        //Thread body
        time = chVTGetSystemTime();
        //detects left right wall
        while(!wall_detected()) {
            left_wall_detected = check_left_sensors();
            right_wall_detected = check_right_sensors();
            front_wall_detected = check_front_sensors();
            if(right_wall_detected && left_wall_detected) {

            }
            set_default_speed();
            set_lr_speed(get_current_speed(), get_current_speed());
        }
        while (!front_wall_detected) {
            front_wall_detected = check_front_sensors();
            if(left_wall_detected) {
                delta_speed = pid_regulator(get_ir_delta(IR6), WALL_TARGET);
                if(fabs(delta_speed) < CORRECTION_THRESHOLD)
                    delta_speed = 0;
                set_lr_speed(get_current_speed() + delta_speed,
                             get_current_speed() - delta_speed);
            }
            else if (right_wall_detected) {
                delta_speed = pid_regulator(get_ir_delta(IR3), WALL_TARGET);
                if(fabs(delta_speed) < CORRECTION_THRESHOLD)
                    delta_speed = 0;
                set_lr_speed(get_current_speed() - delta_speed,
                             get_current_speed() + delta_speed);
            }
            if(check_wall_edge() || front_wall_detected) {
                wall_follower_thd_paused = true;
                chBSemSignal(&wall_edge_detected);
                stop_move();
                break;
            }
        }
        chThdSleepUntilWindowed(time, time + MS2ST(WALL_FOLLOWER_PERIOD));
    }

    wall_follower_thd_created = false;
	chThdExit(0);
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void create_wall_follower_thd(void) {
    if(!wall_follower_thd_created) {
        ptr_wall_follower_thd = chThdCreateStatic(wa_wall_follower_thd,
            sizeof(wa_wall_follower_thd), NORMALPRIO, wall_follower_thd, NULL);
        wall_follower_thd_created = true;
    }
}

void stop_wall_follower_thd(void) {
    if(wall_follower_thd_created) {
        resume_wall_follower_thd();
        chThdTerminate(ptr_wall_follower_thd);
        chThdWait(ptr_wall_follower_thd);
        stop_move();
        wall_follower_thd_created = false;
        wall_follower_thd_paused = false;
    }
}

void pause_wall_follower_thd(void) {
    if (wall_follower_thd_created) wall_follower_thd_paused = true;
}

void resume_wall_follower_thd(void) {
    chSysLock();
	if(wall_follower_thd_created && wall_follower_thd_paused){
		chSchWakeupS(ptr_wall_follower_thd, CH_STATE_READY);
		wall_follower_thd_paused = false;
	}
	chSysUnlock();
}

bool wall_follower_thd_status(void) {
    return wall_follower_thd_created;
}

void move_to_next_wall(void) {
    left_wall_detected = false;
    front_wall_detected = false;
    right_wall_detected = false;
    wall_follower_thd_paused = false;
}

binary_semaphore_t *get_edge_detected_semaphore_ptr(void)
{
	return &wall_edge_detected;
}
