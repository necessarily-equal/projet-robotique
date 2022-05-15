#include <stdint.h>
#include <stdbool.h>

/*
 * Basics
 */

typedef char action_t;

#define ACTION_STRAIGHT     'S'     // go straight forward
#define ACTION_LEFT         'L'     // turn left
#define ACTION_RIGHT        'R'     // turn right
#define ACTION_BACK         'B'     // u turn
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

/*
 * A list of saved actions
 * The list is always null-terminated (ACTION_VOID-terminated) and is of size (SAVED_PATH_SIZE+1)
 * saved_path_push is well-behaved in case the list is already full.
 */

#define SAVED_PATH_SIZE (2<<10)
// delete the saved path so that it is possible to build a new one using saved_path_push.
// (you don't need to call that on first start)
void reset_saved_path(void);
// append an action to the list
// returns true on success
bool saved_path_push(action_t action);
// returns a pointer to the saved path
const action_t *get_saved_path(void);
// returns the saved path, simplified
// `out` must point to a buffer that must be at least (SAVED_PATH_SIZE+1) big.
void get_simplified_saved_path(action_t *out);
