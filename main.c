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
#include <move_command.h>
#include <communication.h>
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

    create_motor_thd();
    set_default_speed();
    sensors_init();
    create_corridor_navigation_thd();
}

/*===========================================================================*/
/* Main function.                                                            */
/*===========================================================================*/

int main(void)
{
    init_all();
    control_maze();
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
