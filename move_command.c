/**
 * @file move_command.c
 * @brief 
 */

// C standard headers
#include <math.h>

// ChibiOS headers
#include "hal.h"
//#include "ch.h"

// e-puck 2 main processor headers
#include <motors.h>

// Module headers
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

//Delay constants
#define MOTOR_DELAY			100

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

//Thread local variables
static bool motor_thd_created = false;	//Thread is created
static bool motor_thd_paused = false;	//Thread is paused

static bool is_moving = false;			//Motor haven't reached target_pos
static int32_t l_target_pos = 0;
static int32_t r_target_pos = 0;
static int16_t current_speed = DEFAULT_SPEED;
static direction_t current_direction = FORWARD;
static bool rotation_enabled = false;

/*===========================================================================*/
/* Semaphores.                                                               */
/*===========================================================================*/

static BSEMAPHORE_DECL(move_finished, TRUE);

/*===========================================================================*/
/* Module thread pointers                                                    */
/*===========================================================================*/

static thread_t *ptr_motor_thd = NULL;

/*===========================================================================*/
/* Module threads.                                                           */
/*===========================================================================*/

static THD_WORKING_AREA(wa_motor_thd, 256);
static THD_FUNCTION(motor_thd, arg) {
	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	systime_t time;
	int32_t l_pos=0;
	int32_t r_pos=0;

	while (!chThdShouldTerminateX()) {
		//Thread Sleep
		chSysLock();
		if (motor_thd_paused) {
			left_motor_set_speed(NULL_SPEED);
			right_motor_set_speed(NULL_SPEED);
			chSchGoSleepS(CH_STATE_SUSPENDED);
		}
		chSysUnlock();
		//Thread body
		l_pos=current_direction * left_motor_get_pos();
		r_pos=current_direction * right_motor_get_pos();

		if (l_pos>l_target_pos && r_pos>r_target_pos && !rotation_enabled) {
			left_motor_set_speed(NULL_SPEED);
			right_motor_set_speed(NULL_SPEED);
			is_moving = false;
			chThdSleepMilliseconds(MOTOR_DELAY);
			chBSemSignal(&move_finished);
		}
		else if (l_pos>l_target_pos && r_pos<r_target_pos && rotation_enabled) {
			left_motor_set_speed(NULL_SPEED);
			right_motor_set_speed(NULL_SPEED);
			is_moving = false;
			chThdSleepMilliseconds(MOTOR_DELAY);
			chBSemSignal(&move_finished);
		}

		//Thread refresh rate
		time = chVTGetSystemTime();
		chThdSleepUntilWindowed(time, time + MS2ST(MOTOR_THD_PERIOD));
	}

	motor_thd_created = false;
	chThdExit(0);
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void create_motor_thd(void) {
	if (!motor_thd_created) {
		motors_init();
		ptr_motor_thd = chThdCreateStatic(wa_motor_thd,
			sizeof(wa_motor_thd), NORMALPRIO, motor_thd, NULL);
		motor_thd_created = true;
	}
}

void stop_motor_thd(void) {
	if (motor_thd_created) {
		resume_motor_thd();
		chThdTerminate(ptr_motor_thd);
		chThdWait(ptr_motor_thd);
		left_motor_set_speed(NULL_SPEED);
		right_motor_set_speed(NULL_SPEED);
		motor_thd_created = false;
		motor_thd_paused = false;
	}
}

void pause_motor_thd(void) {
	if (motor_thd_created) {
		motor_thd_paused = true;
	}
}

void resume_motor_thd(void) {
	chSysLock();
	if (motor_thd_created && motor_thd_paused) {
		chSchWakeupS(ptr_motor_thd, CH_STATE_READY);
		motor_thd_paused = false;
	}
	chSysUnlock();
}

bool motor_is_moving(void) {
	return is_moving;
}

void right_angle_turn(direction_t direction) {
	turn(ADJUSTED_90DEG_TURN, direction);
}

void u_turn(void) {
	turn(ADJUSTED_U_TURN, COUNTERCLOCKWISE);
}

void turn(float position, rotation_t direction) {
	if (motor_thd_created && !motor_thd_paused) {
		if (!is_moving) {
			set_default_speed();
			left_motor_set_pos(0);
			right_motor_set_pos(0);
			l_target_pos=position * WHEEL_TURN_STEPS / WHEEL_PERIMETER;
			r_target_pos=-(position * WHEEL_PERIMETER / WHEEL_PERIMETER);

			left_motor_set_speed(direction*DEFAULT_SPEED);
			right_motor_set_speed(-(direction*DEFAULT_SPEED));
			if(direction == CLOCKWISE)
				current_direction = FORWARD;
			else
				current_direction = BACKWARD;
			rotation_enabled = true;
			is_moving = true;
		}
	}
}

void move(float position, direction_t direction) {
	if (motor_thd_created && !motor_thd_paused) {
		if (!is_moving) {
			set_default_speed();
			left_motor_set_pos(0);
			right_motor_set_pos(0);
			l_target_pos=position * WHEEL_TURN_STEPS / WHEEL_PERIMETER;
			r_target_pos=position * WHEEL_PERIMETER / WHEEL_PERIMETER;

			left_motor_set_speed(direction*DEFAULT_SPEED);
			right_motor_set_speed(direction*DEFAULT_SPEED);
			rotation_enabled = false;
			current_direction = direction;
			is_moving = true;
		}
	}
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

void stop_move(void) {
	is_moving = false;
	current_direction = FORWARD;
	rotation_enabled = false;
	l_target_pos = 0;
	r_target_pos = 0;
}

binary_semaphore_t *get_motor_semaphore_ptr(void) {
	return &move_finished;
}

