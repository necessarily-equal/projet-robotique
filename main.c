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
#include <wall_follower.h>
//#include <maze_control.h>
#include <mic_processing.h>
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
//  usb_start();

    motors_init();
    sensors_init();
    chThdSleepMilliseconds(5000);

    create_wall_follower_thd();
}

/*===========================================================================*/
/* Main function.                                                            */
/*===========================================================================*/

int main(void)
{
    init_all();

    set_default_speed();

    while(true){
        chprintf((BaseSequentialStream *)&SD3, "IR TOF\r\n");
        chprintf((BaseSequentialStream *)&SD3,"%u\n", get_tof_dist());
        chprintf((BaseSequentialStream *)&SD3, "\r\n\n");
        chprintf((BaseSequentialStream *)&SD3, "IR PROXIMITY\r\n");
        chprintf((BaseSequentialStream *)&SD3, "%4d ", get_ir_delta(IR1));
        chprintf((BaseSequentialStream *)&SD3, "%4d ", get_ir_delta(IR2));
        chprintf((BaseSequentialStream *)&SD3, "%4d ", get_ir_delta(IR3));
        chprintf((BaseSequentialStream *)&SD3, "%4d ", get_ir_delta(IR4));
        chprintf((BaseSequentialStream *)&SD3, "%4d ", get_ir_delta(IR5));
        chprintf((BaseSequentialStream *)&SD3, "%4d ", get_ir_delta(IR6));
        chprintf((BaseSequentialStream *)&SD3, "%4d ", get_ir_delta(IR7));
        chprintf((BaseSequentialStream *)&SD3, "%4d ", get_ir_delta(IR8));
        chprintf((BaseSequentialStream *)&SD3, "\r\n\n");
        chThdSleepMilliseconds(MAIN_PERIOD);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}