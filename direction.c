#include <stdbool.h>
#include <stdio.h>

//#define	ASSERT_UNREACHABLE() printf("unreachable, %s at line %i\n", __FUNCTION__, __LINE__)
#define	ASSERT_UNREACHABLE()

static int to_int(char direction) {
	if (direction == ACTION_BACK) return 0;
	if (direction == ACTION_STRAIGHT) return 1;
	if (direction == ACTION_LEFT) return 2;
	if (direction == ACTION_RIGHT) return 3;

	ASSERT_UNREACHABLE();
	return -1;
}

static char table[4][4] =
	{
		{ -1, -1, -1, -1 }, // first is BACK
		{ -1,  0,  3,  2 }, // first is STRAIGHT
		{ -1,  3,  1,  0 }, // first is LEFT
		{ -1,  2,  0,  1 }, // first is RIGHT
	};
static char collapse_three(char a, char b, char c) {
	int first  = to_int(a);
	int second = to_int(b);
	int third  = to_int(c);

	if (second != 0)
		return '\0'; // this function can't simplify this

	int result = table[first][third];

	if (result == 0) return ACTION_BACK;
	if (result == 1) return ACTION_STRAIGHT;
	if (result == 2) return ACTION_LEFT;
	if (result == 3) return ACTION_RIGHT;
	ASSERT_UNREACHABLE();
	return '\0';
}

static bool simplify_once(char *const actions) {
	char *p = actions; // front of the simplified list
	char *q = actions; // front of the original list
	bool needs_another_pass = false;

	char *end = actions;
	while (*end != '\0') end++;
	// end now points to the '\0' at the end

	while (q != end && q != end-1 && q != end-2) {
		char result = collapse_three(*q, *(q+1), *(q+2));
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
	*p = '\0';

	return needs_another_pass;
}

void simplify(char *const actions) {
	while(simplify_once(actions));
}

/*
int main(int argc, char **argv) {
	if (argc <= 1) {
		printf("missing argument");
		return 1;
	}

	simplify(argv[1]);
	printf("result: %s\n", argv[1]);
}
*/
