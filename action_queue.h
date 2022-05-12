typedef action_t char;

#define ACTION_STRAIGHT     'S'     // go straight forward
//#define ACTION_START        'E'     // begin maze navigation
#define ACTION_LEFT         'L'     // turn left
#define ACTION_RIGHT        'R'     // turn right
#define ACTION_BACK         'B'     // u turn
//#define ACTION_END          'D'     // detect the maze exit
#define ACTION_VOID         '\0'    // no action found, this should always terminate the action list

void simplify_action_list(action_t *const actions);
