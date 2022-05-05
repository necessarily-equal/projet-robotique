#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <lfr_regulator.h>
#include <image_processing.h>

/// Update the Line Follower Robot controller with the given line
/// position measurement and return the new control signal.
int16_t lfr_regulator(uint16_t position, uint16_t target){

    float error = 0;
    float speed_correction = 0;
    // integral
    static float sum_error = 0;

    // Error between setpoint and true position
    error = position - target;

    //disables the PI regulator if the error is to small
    //this avoids to always move as we cannot exactly be
    //where we want and the camera is a bit noisy
    if(fabs(error) < ERROR_THRESHOLD){
        return 0;
    }

    sum_error += error;

	// Clamp the output
    if(sum_error > MAX_SUM_ERROR)
        sum_error = MAX_SUM_ERROR;
    else if(sum_error < -MAX_SUM_ERROR)
		sum_error = -MAX_SUM_ERROR;

    // PI formula:
    // u[k] = Kp e[k] + Ki e_i[k], control signal
    speed_correction = KP * error + KI * sum_error;
    // return the control signal
    return (int16_t)speed_correction;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();
        
        //computes the speed correction to apply to the motors
        //distance_cm is modified by the image processing thread
        speed_correction = lfr_regulator(get_line_position(), IMAGE_BUFFER_SIZE/2);

        //if the line is nearly in front of the camera, don't rotate
        if(abs(speed_correction) < ROTATION_THRESHOLD){
        	speed_correction = 0;
        }

        //applies the speed from the PI regulator and the correction for the rotation
		right_motor_set_speed(DEFAULT_SPEED - ROTATION_COEFF * speed_correction);
		left_motor_set_speed(DEFAULT_SPEED + ROTATION_COEFF * speed_correction);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void lfr_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
