// This is a fast and safe interface to the distance sensor.
//
// "Safe" means that it won't tell you that no obstacle is close if the
// mesurement failed. "Fast" means that the data is updated every 20ms, which
// is better than sensors/VL53L0X/VL53L0X.c, which manages to update the
// distance only every 100ms.

void dist_init(void);
bool dist_obstacle_is_close(void);
