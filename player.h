#include <stdint.h>

void leds_init(void);

// if freq == 0, this will switch off leds instead
void leds_set_frequency(uint16_t freq);
