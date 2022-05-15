#include "action_queue.h"
#include "move_command.h"
#include "corridor_navigation.h"
#include "distance.h"
#include "ir_sensors.h"

#include "selector.h"
#include "leds.h"

static void execute_action(action_t action) {
	switch (action) {
	case ACTION_STRAIGHT:
		move(4.0f, FORWARD);
		chBSemWait(get_motor_semaphore_ptr());
		navigate_corridor();
		chBSemWait(get_corridor_end_detected_semaphore_ptr());
		chBSemWait(get_motor_semaphore_ptr());
		move(2.75f, FORWARD);
		chBSemWait(get_motor_semaphore_ptr());
		break;
	case ACTION_BACK:
		u_turn();
		chBSemWait(get_motor_semaphore_ptr());
		action_queue_push(ACTION_STRAIGHT);
		break;
	case ACTION_LEFT:
		right_angle_turn(COUNTERCLOCKWISE);
		chBSemWait(get_motor_semaphore_ptr());
		action_queue_push(ACTION_STRAIGHT);
		break;
	case ACTION_RIGHT:
		right_angle_turn(CLOCKWISE);
		chBSemWait(get_motor_semaphore_ptr());
		action_queue_push(ACTION_STRAIGHT);
		break;
	default:
		break;
	}
}

// this implements a simple left-following maze solving algorithm
static void show_next_actions(void) {
	set_led(3, get_ir_delta(IR6) < 150);
	set_led(0, dist_get_distance() > 80);
	set_led(1, get_ir_delta(IR3) < 150);
}

// this implements a simple left-following maze solving algorithm
static action_t find_next_action(void) {
	bool is_auto_feature_enabled = (get_selector() % 4) <= 1;
	if (!is_auto_feature_enabled)
		return ACTION_VOID;

	if (get_ir_delta(IR6) < 150)
		return ACTION_LEFT;
	if (dist_get_distance() > 80)
		return ACTION_STRAIGHT;
	if (get_ir_delta(IR3) < 150)
		return ACTION_RIGHT;
	return ACTION_BACK;
}

void control_maze(void) {
	show_next_actions();

	action_t current_action = ACTION_VOID;
	if (!(current_action = action_queue_pop())) {
		if (!(current_action = find_next_action())) {
			// signal that we are stuck
			set_front_led(1);
			chThdSleepMilliseconds(10);
		}
	}

	set_front_led(0);

	// save and execute this action
	saved_path_push(current_action);
	execute_action(current_action);
}
