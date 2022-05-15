/**
 * @file move_command.c
 * @brief 
 * 
 */

//C standard headers
#include <math.h>

//ChibiOS headers
#include "hal.h"
#include "ch.h"
#include "chschd.h"

//E-puck 2 main processor headers
#include "motors.h"
#include "leds.h"

//Module headers
#include "ir_sensors.h"
#include "move_command.h"

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/
//Geometric constants
#define PI                  	3.1415926536f
#define WHEEL_SEPARATION    	5.35f   // [cm]
#define WHEEL_PERIMETER     	13      // [cm]
#define EPUCK_PERIMETER     	(PI*WHEEL_SEPARATION)
//Adjusted manoeuvre based on experiments
#define ADJUSTED_U_TURN			(EPUCK_PERIMETER/2)*0.98075
#define ADJUSTED_90DEG_TURN 	ADJUSTED_U_TURN/2
//Speed constants
#define NULL_SPEED          0
#define DEFAULT_SPEED		500
#define MAX_SPEED			800
//Steppers constants
#define WHEEL_TURN_STEPS    1000    //Number of steps for one turn
//Wall collision
#define WALL_THLD			1500
//Thread constants
#define MOTOR_THD_PERIOD	100

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static bool is_moving = false;
static bool wall_ahead = false;
static bool rotation_mode = false;
static bool motor_thd_paused = false;

static int32_t l_target_pos = 0;
static int32_t r_target_pos = 0;
static int32_t l_pos = 0;
static int32_t r_pos = 0;

static direction_t current_direction = FORWARD;
static rotation_t current_rotation = COUNTERCLOCKWISE;

/*===========================================================================*/
/* Semaphores.                                                               */
/*===========================================================================*/

static BSEMAPHORE_DECL(move_command_finished, TRUE);

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

bool collision(void) {
	//Front
	if(get_ir_delta(IR1) > WALL_THLD) return true;
	if(get_ir_delta(IR8) > WALL_THLD) return true;
	//Rear
	//if(get_ir_delta(IR4) > WALL_THLD) return true;
	//if(get_ir_delta(IR5) > WALL_THLD) return true;
	return false;
}

void update_current_position(void) {
	if (rotation_mode) {
		l_pos = current_rotation * left_motor_get_pos();
		r_pos = current_rotation * right_motor_get_pos();
	}
	else {
		l_pos = current_direction * left_motor_get_pos();
		r_pos = current_direction * right_motor_get_pos();
	}
}

bool position_reached(void) {
	if (l_pos>l_target_pos && r_pos>r_target_pos && !rotation_mode)
		return true;
	if (l_pos>l_target_pos && r_pos<r_target_pos && rotation_mode)
		return true;
	return false;
}

void stop_moving(void) {
	left_motor_set_speed(NULL_SPEED);
	right_motor_set_speed(NULL_SPEED);
	current_direction = FORWARD;
	current_rotation = COUNTERCLOCKWISE;
	rotation_mode = false;
	l_target_pos = 0;
	r_target_pos = 0;
	is_moving = false;
	chBSemSignal(&move_command_finished);
}

/*===========================================================================*/
/* Module threads.                                                           */
/*===========================================================================*/

static THD_WORKING_AREA(wa_motor_thd, 256);
static THD_FUNCTION(motor_thd, arg) {
	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	systime_t time;

	while (true) {
		wall_ahead = collision();
		if (wall_ahead) stop_moving();
		else {
			if (motor_thd_paused) {
				left_motor_set_speed(NULL_SPEED);
				right_motor_set_speed(NULL_SPEED);
			}
			update_current_position();
			if (position_reached()) stop_moving();
		}
		time = chVTGetSystemTime();
		chThdSleepUntilWindowed(time, time + MS2ST(MOTOR_THD_PERIOD));
	}

	chThdExit(0);
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void init_motors_thd(void) {
	motors_init();
	chThdCreateStatic(wa_motor_thd, sizeof(wa_motor_thd), NORMALPRIO + 1,
					  motor_thd, NULL);
}

void pause_motor_thd(void) {
	motor_thd_paused = true;
}

void resume_motor_thd(void) {
	motor_thd_paused = false;
}

bool motor_thd_status(void) {
	return motor_thd_paused;
}

bool get_is_moving(void) {
	return is_moving;
}

void turn(float position, rotation_t direction) {
	motor_thd_paused = false;
	if (!is_moving && !wall_ahead) {
		left_motor_set_pos(0);
		right_motor_set_pos(0);
		l_target_pos = position * WHEEL_TURN_STEPS / WHEEL_PERIMETER;
		r_target_pos = -(position * WHEEL_PERIMETER / WHEEL_PERIMETER);
		current_rotation = direction;
		rotation_mode = true;
		is_moving = true;
		left_motor_set_speed(direction * DEFAULT_SPEED);
		right_motor_set_speed(-(direction * DEFAULT_SPEED));
	}
}

void move(float position, direction_t direction) {
	motor_thd_paused = false;
	if (!is_moving && !wall_ahead) {
		left_motor_set_pos(0);
		right_motor_set_pos(0);
		l_target_pos=position * WHEEL_TURN_STEPS / WHEEL_PERIMETER;
		r_target_pos=position * WHEEL_PERIMETER / WHEEL_PERIMETER;
		current_direction = direction;
		rotation_mode = false;
		is_moving = true;
		left_motor_set_speed(direction*DEFAULT_SPEED);
		right_motor_set_speed(direction*DEFAULT_SPEED);
	}
}

void right_angle_turn(direction_t direction) {
	turn(ADJUSTED_90DEG_TURN, direction);
}

void u_turn(void) {
	turn(ADJUSTED_U_TURN, COUNTERCLOCKWISE);
}

void set_lr_speed(int left_speed, int right_speed) {
	if (left_speed > MAX_SPEED)
		left_motor_set_speed(MAX_SPEED);
	else if (left_speed < -MAX_SPEED)
		left_motor_set_speed(-MAX_SPEED);
	else
		left_motor_set_speed(left_speed);

	if (right_speed > MAX_SPEED)
		right_motor_set_speed(MAX_SPEED);
	else if (right_speed < -MAX_SPEED)
		right_motor_set_speed(-MAX_SPEED);
	else
		right_motor_set_speed(right_speed);
}

binary_semaphore_t *get_motor_semaphore_ptr(void) {
	return &move_command_finished;
}