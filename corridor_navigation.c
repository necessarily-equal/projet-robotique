/**
 * @file corridor_navigation.c
 * @brief 
 */

// C standard headers
#include <math.h>

// ChibiOS headers
#include "hal.h"
#include "ch.h"
#include "chschd.h"

// e-puck 2 main processor headers
#include "sensors/proximity.h"
#include <chprintf.h>//rm if not using bluetooth

// Module headers
#include "corridor_navigation.h"
#include "ir_sensors.h"
#include "move_command.h"

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

//PID constants.
#define SIDE_WALL_TARGET_VALUE      1000

#define CORRECTION_THLD             1
#define LINK_ERROR_THRESHOLD        0.1f
#define LINK_UPPER_CLAMP            50
#define LINK_LOWER_CLAMP            -50
#define LINK_KP                     0.01f
#define LINK_KI                     0.01f
#define LINK_KD                     5.0f
#define MAX_SUM_ERROR 			    100
//Walls constants.
#define WALL_EDGE_THLD              100 // IR3 & IR6
#define FRONT_WALL_THLD             1500 // IR1 & IR8
#define FRONT_SIDE_WALL_THLD        1000 // IR2 & IR7
//Thread constants.
#define CORRIDOR_NAV_THD_PERIOD     100

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static bool corridor_nav_thd_paused = true;
static bool front_wall_detected = false;
static bool corridor_end_detected = false;

/*===========================================================================*/
/* Semaphores.                                                               */
/*===========================================================================*/

static BSEMAPHORE_DECL(corridor_end_detected_semaphore, TRUE);

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

bool check_front_sensors(void) {
    //Front checks
    if(get_ir_delta(IR1) > FRONT_WALL_THLD) return true;
    if(get_ir_delta(IR8) > FRONT_WALL_THLD) return true;
    //Front left and right checks, too noisy !
    //if(get_ir_delta(IR2) > FRONT_SIDE_WALL_THLD) return true;
    //if(get_ir_delta(IR7) > FRONT_SIDE_WALL_THLD) return true;
    return false;
}

bool check_corridor_end(void) {
    if(get_ir_delta(IR3) < WALL_EDGE_THLD) return true;
    if(get_ir_delta(IR6) < WALL_EDGE_THLD) return true;
    return false;
}

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

    if(fabs(error) < LINK_ERROR_THRESHOLD) return 0;
    //Calculate the integral
    sum_error += error;
    //Calcualte the derivative
    derivative = error - last_error;

    if(sum_error > MAX_SUM_ERROR) sum_error = MAX_SUM_ERROR;
	else if(sum_error < -MAX_SUM_ERROR) sum_error = -MAX_SUM_ERROR;

    last_error = error;
    control = LINK_KP * error + LINK_KI * sum_error + LINK_KD * derivative;
    //output clamping
    if(control > LINK_UPPER_CLAMP) control = LINK_UPPER_CLAMP;
	else if(control < LINK_LOWER_CLAMP) control = LINK_LOWER_CLAMP;

    return (int16_t)control;
}

void corridor_pid_control(void) {
    int32_t delta_speed = 0;
    int32_t left_right_ir_delta = 0;

    left_right_ir_delta = get_ir_delta(IR6) - get_ir_delta(IR3);
    delta_speed = pid_regulator(left_right_ir_delta, 0.0);

    if(fabs(delta_speed) < CORRECTION_THLD) delta_speed = 0;

    set_lr_speed(DEFAULT_SPEED + delta_speed,
                 DEFAULT_SPEED - delta_speed);
}

/*===========================================================================*/
/* Module threads.                                                           */
/*===========================================================================*/

static THD_WORKING_AREA(wa_corridor_nav_thd, 1024);
static THD_FUNCTION(corridor_nav_thd, arg)
{
    chRegSetThreadName(__FUNCTION__);
	(void)arg;

    systime_t time;

    while (!chThdShouldTerminateX()) {
        //Thread body
        while (!front_wall_detected && !corridor_end_detected) {
            corridor_end_detected = check_corridor_end();
            corridor_pid_control();
        }
        chBSemSignal(&corridor_end_detected_semaphore);
        corridor_nav_thd_paused = true;
        //Thread refresh rate
        time = chVTGetSystemTime();
        chThdSleepUntilWindowed(time, time + MS2ST(CORRIDOR_NAV_THD_PERIOD));
    }
	chThdExit(0);
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void create_corridor_navigation_thd(void) {
    chThdCreateStatic(wa_corridor_nav_thd, sizeof(wa_corridor_nav_thd),
                      NORMALPRIO, corridor_nav_thd, NULL);
}

void disable_corridor_navigation_thd(void) {
    corridor_nav_thd_paused = true;
}

bool corridor_navigation_thd_status(void) {
    return corridor_nav_thd_paused;
}

void navigate_corridor(void) {
    corridor_nav_thd_paused = false;
    front_wall_detected = false;
    corridor_end_detected = false;
}

binary_semaphore_t *get_corridor_end_detected_semaphore_ptr(void) {
    return &corridor_end_detected_semaphore;
}
