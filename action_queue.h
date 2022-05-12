/*
 * Basics
 */

typedef char action_t;

#define ACTION_STRAIGHT     'S'     // go straight forward
//#define ACTION_START        'E'     // begin maze navigation
#define ACTION_LEFT         'L'     // turn left
#define ACTION_RIGHT        'R'     // turn right
#define ACTION_BACK         'B'     // u turn
//#define ACTION_END          'D'     // detect the maze exit
#define ACTION_VOID         '\0'    // no action found, this should always terminate the action list


/*
 * Combining actions to condense a path into a simpler one that goes to the same,
 * but may be better optimised.
 */

// This modifies the `actions` parameter in-place.
// This assumes the path is acyclic.
void simplify_action_list(action_t *const actions);


/*
 * A circular buffer to hold the actions to come
 */

// Is the queue full or empty?
// FIXME: decide if we need these two
bool action_queue_empty(void);
bool action_queue_full(void);
// Append an action at the end of the queue
void action_queue_push(action_t action);
// Returns the first action of the queue and removes it from the queue.
// If the queue is empty, returns ACTION_VOID
action_t action_queue_pop(void);
