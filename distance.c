#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "ch.h"
#include "hal.h"

#include <sensors/VL53L0X/VL53L0X.h>

#include "distance.h"

static const uint16_t hysteresis_close_threshold = 40;
static const uint16_t hysteresis_far_threshold = 60;

//static BSEMAPHORE_DECL(obstacle_is_close, true);
static bool obstacle_is_close = true;

static VL53L0X_Dev_t device;

static THD_WORKING_AREA(waDistanceThd, 256);
static THD_FUNCTION(DistanceThd, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg; // avoid warning for unused argument

    while (chThdShouldTerminateX() == false) {

	//uint16_t distance = VL53L0X_get_dist_mm();

        // this updates device with the mesured range, if and only if the mesurement works
        VL53L0X_Error error = VL53L0X_getLastMeasure(&device);
	if (error == VL53L0X_ERROR_NONE) {
	    uint16_t distance = device.Data.LastRangeMeasure.RangeMilliMeter;

            if (obstacle_is_close && distance > hysteresis_far_threshold)
                obstacle_is_close = false;
            else if (!obstacle_is_close && distance < hysteresis_close_threshold)
                obstacle_is_close = true;
	}

        chThdSleepMilliseconds(20);
    }
}

void dist_init(void) {
    device.I2cDevAddr = VL53L0X_ADDR;

    VL53L0X_init(&device);
    VL53L0X_configAccuracy(&device, VL53L0X_HIGH_SPEED);
    VL53L0X_startMeasure(&device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

    chThdCreateStatic(waDistanceThd, sizeof(waDistanceThd), NORMALPRIO, DistanceThd, NULL);
}

bool dist_obstacle_is_close(void) {
    return obstacle_is_close;
}
