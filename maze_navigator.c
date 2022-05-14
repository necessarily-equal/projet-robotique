#include "action_queue.h"
#include "move_command.h"
#include "wall_follower.h"
#include "ir_sensors.h"

static void dispatch_action(action_t action) {
	switch (action) {
	case ACTION_STRAIGHT:
		// TODO: call forward
		break;
	case ACTION_BACK:
		u_turn();
		break;
	case ACTION_LEFT:
		right_angle_turn(COUNTERCLOCKWISE);
		break;
	case ACTION_RIGHT:
		right_angle_turn(CLOCKWISE);
		break;
	default:
		break;
	}
}

static void wait_action(action_t action) {
	switch(action) {
	case ACTION_STRAIGHT:
		chBSemWait(get_edge_detected_semaphore_ptr());
		break;
	case ACTION_BACK:
		chBSemWait(get_motor_semaphore_ptr());
		break;
	case ACTION_LEFT:
		chBSemWait(get_motor_semaphore_ptr());
		break;
	case ACTION_RIGHT:
		chBSemWait(get_motor_semaphore_ptr());
		break;
	}
}

// this implements a simple left-following maze solving algorithm
static action_t find_next_action(void) {
	if (get_ir_delta(IR6) < 100)
		return ACTION_LEFT;
	if (get_tof_dist() > 300)
		return ACTION_STRAIGHT;
	if (get_ir_delta(IR3) > 100)
		return ACTION_RIGHT;
	return ACTION_BACK;
}

void control_maze(void) {
	while (1) {
		action_t current_action = action_queue_pop();
		if (!current_action) {
			current_action = find_next_action();
		}

		saved_path_push(current_action);
		dispatch_action(current_action);
		wait_action(current_action);
	}
}
