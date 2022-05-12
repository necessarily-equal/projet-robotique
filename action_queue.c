#include <stdbool.h>
#include <stdio.h>

#include <ch.h>

#include "action_queue.h"

//#define	ASSERT_UNREACHABLE() printf("unreachable, %s at line %i\n", __FUNCTION__, __LINE__)
#define	ASSERT_UNREACHABLE()

static int to_int(action_t direction) {
	if (direction == ACTION_BACK) return 0;
	if (direction == ACTION_STRAIGHT) return 1;
	if (direction == ACTION_LEFT) return 2;
	if (direction == ACTION_RIGHT) return 3;

	ASSERT_UNREACHABLE();
	return -1;
}

static const int table[4][4] =
	{
		{ -1, -1, -1, -1 }, // first is BACK
		{ -1,  0,  3,  2 }, // first is STRAIGHT
		{ -1,  3,  1,  0 }, // first is LEFT
		{ -1,  2,  0,  1 }, // first is RIGHT
	};
static action_t collapse_three_actions(action_t a, action_t b, action_t c) {
	int first  = to_int(a);
	int second = to_int(b);
	int third  = to_int(c);

	if (second != 0)
		return ACTION_VOID; // this function can't simplify this

	int result = table[first][third];

	if (result == 0) return ACTION_BACK;
	if (result == 1) return ACTION_STRAIGHT;
	if (result == 2) return ACTION_LEFT;
	if (result == 3) return ACTION_RIGHT;
	ASSERT_UNREACHABLE();
	return ACTION_VOID;
}

static bool simplify_once(action_t *const actions) {
	char *p = actions; // front of the simplified list
	char *q = actions; // front of the original list
	bool needs_another_pass = false;

	char *end = actions;
	while (*end != ACTION_VOID) end++;
	// end now points to the ACTION_VOID at the end

	while (q != end && q != end-1 && q != end-2) {
		char result = collapse_three_actions(*q, *(q+1), *(q+2));
		if (result) {
			*p++ = result;
			q+=3;

			if (result == ACTION_BACK)
				needs_another_pass = true;
		} else {
			*p++ = *q++;
		}
	}

	// nothing else we can collapse, let's just copy
	while (q != end)
		*p++ = *q++;
	*p = ACTION_VOID;

	return needs_another_pass;
}

void simplify_action_list(action_t *const actions) {
	while(simplify_once(actions));
}


/*
 * Event Queue
 */

#define ACTION_QUEUE_SIZE (1<<5)
#define ACTION_QUEUE_MASK (ACTION_QUEUE_SIZE - 1)

MUTEX_DECL(action_queue_mutex);

unsigned action_queue_front;
unsigned action_queue_back;
action_t action_queue[ACTION_QUEUE_SIZE];

bool action_queue_empty(void) {
	chMtxLock(&action_queue_mutex);
	bool res = action_queue_front == action_queue_back;
	chMtxUnlock(&action_queue_mutex);
	return res;
}

bool action_queue_full(void) {
	chMtxLock(&action_queue_mutex);
	bool res = ((action_queue_back+1) & ACTION_QUEUE_MASK) == action_queue_front;
	chMtxUnlock(&action_queue_mutex);
	return res;
}

void action_queue_push(action_t action) {
	chMtxLock(&action_queue_mutex);
	if (((action_queue_back+1) & ACTION_QUEUE_MASK) == action_queue_front) {
		// the queue is already full
		chMtxUnlock(&action_queue_mutex);
		return;
	}

	action_queue[action_queue_back] = action;
	action_queue_back += 1;
	action_queue_back &= ACTION_QUEUE_MASK;

	chMtxUnlock(&action_queue_mutex);
}

action_t action_queue_pop(void) {
	chMtxLock(&action_queue_mutex);
	if (action_queue_front == action_queue_back) {
		// we have nothing to pop
		chMtxUnlock(&action_queue_mutex);
		return ACTION_VOID;
	}

	action_t action = action_queue[action_queue_front];
	action_queue_front += 1;
	action_queue_front &= ACTION_QUEUE_MASK;

	chMtxUnlock(&action_queue_mutex);
	return action;
}
