#include "action_queue.h"
#include "move_command.h"

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

static action_t find_next_action(void) {
	static int count = 0;
	if (count < 3) {
		count++;
		return ACTION_RIGHT;
	}
	return ACTION_VOID;
}

void control_maze(void) {
	action_queue_push(ACTION_LEFT);
	action_queue_push(ACTION_LEFT);
	action_queue_push(ACTION_LEFT);
	while (1) {
		action_t current_action = ACTION_VOID;
		if (action_queue_empty()) {
			current_action = find_next_action();
		}

		if (!current_action) {
			current_action = action_queue_pop();
		}

		if (current_action) {
			saved_path_push(current_action);
			dispatch_action(current_action);
			chBSemWait(get_motor_sempahore_ptr());
		}
	}
}
