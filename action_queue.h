typedef char action_t;

#define ACTION_STRAIGHT     'S'     // go straight forward
//#define ACTION_START        'E'     // begin maze navigation
#define ACTION_LEFT         'L'     // turn left
#define ACTION_RIGHT        'R'     // turn right
#define ACTION_BACK         'B'     // u turn
//#define ACTION_END          'D'     // detect the maze exit
#define ACTION_VOID         '\0'    // no action found, this should always terminate the action list

void simplify_action_list(action_t *const actions);

bool action_queue_empty(void);
bool action_queue_full(void);
void action_queue_push(action_t action);
action_t action_queue_pop(void);
