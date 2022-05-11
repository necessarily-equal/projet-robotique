/**
 * @file move_command.c
 * @brief 
 */

// C standard headers
#include <stdio.h>
#include <string.h>
#include <math.h>

// ChibiOS headers
#include "hal.h"
#include "ch.h"
#include "memory_protection.h"
#include "msgbus/messagebus.h"

// e-puck 2 main processor headers
#include <motors.h>

// Module headers
#include <move_command.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

//Geometric parameters
#define PI                  3.1415926536f
#define WHEEL_SEPARATION    5.35f   // [cm]
#define WHEEL_PERIMETER     13      // [cm]
#define EPUCK_PERIMETER     (PI * WHEEL_SEPARATION)
#define U_TURN              EPUCK_PERIMETER/2

//Speed
#define TURNING_SPEED       500
#define DEFAULT_SPEED		500
#define NULL_SPEED          0
#define MAX_SPEED			800

//Steppers constants
#define WHEEL_TURN_STEPS    1000    //Number of steps for one turn
#define ADJUSTED_90DEG_TURN 3.8     // Adjusted value based on experiments

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static int16_t current_speed = DEFAULT_SPEED;

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void right_angle_turn(rotation_t direction){
    turn(ADJUSTED_90DEG_TURN, direction);
}

void u_turn(void){
    turn(U_TURN, COUNTERCLOCKWISE);
}

void turn(float position, rotation_t direction){
    int32_t final_l_pos=0;
	int32_t final_r_pos=0;
	int32_t l_pos=0;
	int32_t r_pos=0;

    left_motor_set_pos(0);
	right_motor_set_pos(0);

    final_l_pos=position * WHEEL_TURN_STEPS / WHEEL_PERIMETER;
	final_r_pos=-(position * WHEEL_PERIMETER / WHEEL_PERIMETER);

    left_motor_set_speed(direction*TURNING_SPEED);
	right_motor_set_speed(-(direction*TURNING_SPEED));

    l_pos=direction*left_motor_get_pos();
	r_pos=direction*right_motor_get_pos();

	while(l_pos<final_l_pos || r_pos>final_r_pos ){
		l_pos=direction*left_motor_get_pos();
		r_pos=direction*right_motor_get_pos();
	}

	left_motor_set_speed(NULL_SPEED);
	right_motor_set_speed(NULL_SPEED);
}

void move(float position, direction_t direction){
    int32_t final_l_pos=0;
	int32_t final_r_pos=0;
	int32_t l_pos=0;
	int32_t r_pos=0;

    left_motor_set_pos(0);
	right_motor_set_pos(0);

    final_l_pos=position * WHEEL_TURN_STEPS / WHEEL_PERIMETER;
	final_r_pos=position * WHEEL_PERIMETER / WHEEL_PERIMETER;

    left_motor_set_speed(direction*current_speed);
	right_motor_set_speed(direction*current_speed);

    l_pos=direction*left_motor_get_pos();
	r_pos=direction*right_motor_get_pos();

	while(l_pos<final_l_pos || r_pos>final_r_pos ){
		l_pos=direction*left_motor_get_pos();
		r_pos=direction*right_motor_get_pos();
	}

	left_motor_set_speed(NULL_SPEED);
	right_motor_set_speed(NULL_SPEED);
}

void set_default_speed(void){
	current_speed = DEFAULT_SPEED;
}

void set_current_speed(int16_t new_speed)
{
	if(new_speed > MAX_SPEED)
		current_speed = MAX_SPEED;
	else if(-new_speed < MAX_SPEED)
		current_speed = -MAX_SPEED;
	else
		current_speed = new_speed;
}

int16_t get_current_speed(void)
{
	return current_speed;
}

void set_lr_speed(int left_speed, int right_speed)
{

	if(left_speed > MAX_SPEED)
		left_motor_set_speed(MAX_SPEED);
	else if(left_speed < -MAX_SPEED)
		left_motor_set_speed(-MAX_SPEED);
	else
		left_motor_set_speed(left_speed);

	if(right_speed > MAX_SPEED)
		right_motor_set_speed(MAX_SPEED);
	else if(right_speed < -MAX_SPEED)
		right_motor_set_speed(-MAX_SPEED);
	else
		right_motor_set_speed(right_speed);
}

void stop_move(void){
    current_speed = NULL_SPEED;
    left_motor_set_speed(NULL_SPEED);
	right_motor_set_speed(NULL_SPEED);
}