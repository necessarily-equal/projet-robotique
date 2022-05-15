#include "action_queue.h"
#include "move_command.h"
#include "corridor_navigation.h"
#include "ir_sensors.h"

#include "leds.h"

static void execute_action(action_t action) {
	switch (action) {
	case ACTION_STRAIGHT:
		move(6.0f, FORWARD);
		chBSemWait(get_motor_semaphore_ptr());
		navigate_corridor();
		chBSemWait(get_corridor_end_detected_semaphore_ptr());
		move(2.5f, FORWARD);
		chBSemWait(get_motor_semaphore_ptr());
		break;
	case ACTION_BACK:
		u_turn();
		chBSemWait(get_motor_semaphore_ptr());
		break;
	case ACTION_LEFT:
		right_angle_turn(COUNTERCLOCKWISE);
		chBSemWait(get_motor_semaphore_ptr());
		break;
	case ACTION_RIGHT:
		right_angle_turn(CLOCKWISE);
		chBSemWait(get_motor_semaphore_ptr());
		break;
	default:
		break;
	}
}

// this implements a simple left-following maze solving algorithm
static action_t show_next_actions(void) {
	e_set_led(6, get_ir_delta(IR6) < 150);
	e_set_led(0, get_tof_dist() > 300);
	e_set_led(2, get_ir_delta(IR3) < 150);
}

// this implements a simple left-following maze solving algorithm
static action_t find_next_action(void) {
	if (get_ir_delta(IR6) < 150)
		return ACTION_LEFT;
	if (get_tof_dist() > 300)
		return ACTION_STRAIGHT;
	if (get_ir_delta(IR3) < 150)
		return ACTION_RIGHT;
	return ACTION_BACK;
}

void control_maze(void) {
	action_queue_push(ACTION_STRAIGHT);
	action_queue_push(ACTION_RIGHT);
	//action_queue_push(ACTION_STRAIGHT);
	//action_queue_push(ACTION_LEFT);
	//action_queue_push(ACTION_STRAIGHT);
	while (true) {
		action_t current_action = ACTION_VOID;
		if (!(current_action = action_queue_pop())) {
			//if (!(current_action = find_next_action())) {
				// signal that we are stuck
				set_front_led(1);
				//// here's a sleep so we avoid crashing
				//chThdSleepMilliseconds(100);
				//continue;

				while(true) {
					show_next_actions();
					chThdSleepMilliseconds(10);
				}
			}
		//}

		set_front_led(0);
		// save and execute this action
		saved_path_push(current_action);
		execute_action(current_action);
	}
}
