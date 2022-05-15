/**
 * @file main.c
 * @brief 
 */

// ChibiOS headers

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <motors.h>

// e-puck 2 main processor headers

#include <camera/po8030.h>
#include "camera/dcmi_camera.h"

#include "selector.h"


// Module headers
#include <main.h>
#include <ir_sensors.h>
#include <corridor_navigation.h>
//#include <maze_control.h>
#include <mic_remote_control.h>
#include <maze_navigator.h>
#include <move_command.h>
#include <communication.h>
#include <action_queue.h>
//#include <lfr_regulator.h>
//#include <image_processing.h>

//#include <operation_mode.h>

//#include <distance.h>
//#include <player.h>

/*===========================================================================*/
/* Module constants                                                          */
/*===========================================================================*/

#define MAIN_PERIOD     100 // in milliseconds

/*===========================================================================*/
/* Bus related declarations.                                                 */
/*===========================================================================*/
/*
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);
*/
/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

static void init_all(void){
    halInit();
    chSysInit();
    mpu_init();

    com_serial_start();
//  usb_start(); if not using bluetooth

    create_mic_selector_thd();

    init_motors_thd();
    dist_init();
    sensors_init();
    create_corridor_navigation_thd();
}

static bool check_asks_for_replay_of_saved_actions(void) {
	bool is_on = get_selector() >= 8;
	static bool saved_value = true;
	if (is_on && !saved_value) {
		saved_value = true;
		return true;
	}
	return false;
}

/*===========================================================================*/
/* Main function.                                                            */
/*===========================================================================*/

int main(void)
{
	init_all();
	chThdSleepMilliseconds(2000);
	while (true) {
		if (check_asks_for_replay_of_saved_actions()) {
			static action_t saved_path[SAVED_PATH_SIZE+1];
			// reset saved path and enqueue the saved actions
			get_simplified_saved_path(saved_path);
			reset_saved_path();
			for (action_t *action = saved_path; *action; action++)
				action_queue_push(*action);
		}
		control_maze();
	}
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
