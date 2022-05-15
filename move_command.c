/**
 * @file move_command.c
 * @brief 
 * 
 */

// C standard headers
#include <math.h>

// ChibiOS headers
#include "hal.h"
#include "ch.h"

// e-puck 2 main processor headers
#include <motors.h>
#include "leds.h"

// Module headers
#include "ir_sensors.h"
#include <move_command.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/
//Geometric constants
#define PI                  	3.1415926536f
#define WHEEL_SEPARATION    	5.35f   // [cm]
#define WHEEL_PERIMETER     	13      // [cm]
#define EPUCK_PERIMETER     	(PI*WHEEL_SEPARATION)
//Manoeuvre adjusted values based on experiments
#define ADJUSTED_U_TURN			(EPUCK_PERIMETER/2)*0.98075
#define ADJUSTED_90DEG_TURN 	ADJUSTED_U_TURN/2
//Speed constants
#define NULL_SPEED          0
#define DEFAULT_SPEED		500
#define MAX_SPEED			800		//Sufficent for our application
//Steppers constants
#define WHEEL_TURN_STEPS    1000    //Number of steps for one turn
//Thread constants
#define MOTOR_THD_PERIOD	100

#define FRONT_WALL_THLD             1500 // IR1 & IR8
#define FRONT_SIDE_WALL_THLD        1000 // IR2 & IR7

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static bool is_moving = false;
static bool wall_ahead = false;

static int32_t l_target_pos = 0;
static int32_t r_target_pos = 0;
static int32_t l_pos = 0;
static int32_t r_pos = 0;
static int32_t current_speed = DEFAULT_SPEED;

static direction_t current_direction = FORWARD;
static rotation_t current_rotation = COUNTERCLOCKWISE;
static bool rotation_enabled = false;

//Thread local variables
static bool motor_thd_disabled = false;	//Thread is paused

/*===========================================================================*/
/* Semaphores.                                                               */
/*===========================================================================*/

static BSEMAPHORE_DECL(move_finished, TRUE);

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

bool collision_ahead(void) {
	//Front checks
    if(get_ir_delta(IR1) > FRONT_WALL_THLD) return true;
    if(get_ir_delta(IR8) > FRONT_WALL_THLD) return true;
    //Front left and right checks
    if(get_ir_delta(IR2) > FRONT_SIDE_WALL_THLD) return true;
    if(get_ir_delta(IR7) > FRONT_SIDE_WALL_THLD) return true;
    return false;
}

void update_current_position(void) {
	if (rotation_enabled) {
		l_pos = current_rotation * left_motor_get_pos();
		r_pos = current_rotation * right_motor_get_pos();
	}
	else {
		l_pos = current_direction * left_motor_get_pos();
		r_pos = current_direction * right_motor_get_pos();
	}
}

bool position_reached(void) {
	if (l_pos>l_target_pos && r_pos>r_target_pos && !rotation_enabled)
		return true;
	if (l_pos>l_target_pos && r_pos<r_target_pos && rotation_enabled)
		return true;
	return false;
}

void stop_moving(void) {
	left_motor_set_speed(NULL_SPEED);
	right_motor_set_speed(NULL_SPEED);
	chBSemSignal(&move_finished);
	current_direction = FORWARD;
	current_rotation = COUNTERCLOCKWISE;
	rotation_enabled = false;
	l_target_pos = 0;
	r_target_pos = 0;
	is_moving = false;
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
		wall_ahead = collision_ahead();
		if (wall_ahead) stop_moving();
		else {
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

void init_motor_thd(void) {
	motors_init();
	chThdCreateStatic(wa_motor_thd, sizeof(wa_motor_thd), NORMALPRIO,
					  motor_thd, NULL);
}

void pause_motor_thd(void) {
	set_current_speed(NULL_SPEED);
	motor_thd_disabled = true;
}

void resume_motor_thd(void) {
	set_default_speed();
	motor_thd_disabled = false;
}

bool motor_thd_status(void) {
	return motor_thd_disabled;
}

bool get_is_moving(void) {
	return is_moving;
}

void turn(float position, rotation_t direction) {
	motor_thd_disabled = false;
	if (!is_moving && !wall_ahead) {
		left_motor_set_pos(0);
		right_motor_set_pos(0);
		l_target_pos = position * WHEEL_TURN_STEPS / WHEEL_PERIMETER;
		r_target_pos = -(position * WHEEL_PERIMETER / WHEEL_PERIMETER);
		left_motor_set_speed(direction * current_speed);
		right_motor_set_speed(-(direction * current_speed));
		current_rotation = direction;
		rotation_enabled = true;
		is_moving = true;
	}
}

void move(float position, direction_t direction) {
	motor_thd_disabled = false;
	if (!is_moving && !wall_ahead) {
		left_motor_set_pos(0);
		right_motor_set_pos(0);
		l_target_pos=position * WHEEL_TURN_STEPS / WHEEL_PERIMETER;
		r_target_pos=position * WHEEL_PERIMETER / WHEEL_PERIMETER;
		left_motor_set_speed(direction*current_speed);
		right_motor_set_speed(direction*current_speed);
		current_direction = direction;
		rotation_enabled = false;
		is_moving = true;
	}
}

void right_angle_turn(direction_t direction) {
	turn(ADJUSTED_90DEG_TURN, direction);
}

void u_turn(void) {
	turn(ADJUSTED_U_TURN, COUNTERCLOCKWISE);
}

void set_default_speed(void) {
	current_speed = DEFAULT_SPEED;
}

void set_current_speed(int16_t new_speed) {
	if (new_speed > MAX_SPEED)
		current_speed = MAX_SPEED;
	else if (-new_speed < MAX_SPEED)
		current_speed = -MAX_SPEED;
	else
		current_speed = new_speed;
}

int16_t get_current_speed(void) {
	return current_speed;
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
	return &move_finished;
}

