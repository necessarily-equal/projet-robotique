/**
 * @file main.c
 * @brief 
 */

// C standard headers

// ChibiOS headers

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <chprintf.h>
#include <usbcfg.h>

// e-puck 2 main processor headers

#include <camera/po8030.h>
#include "camera/dcmi_camera.h"

#include <motors.h>

// Module headers
#include <main.h>

#include <operation_mode.h>

//#include <lfr_regulator.h>
//#include <image_processing.h>
//#include <distance.h>
//#include <player.h>
//#include <maze_control.h>
#include <mic_processing.h>
#include <move_command.h>

/*===========================================================================*/
/* Module constants                                                          */
/*===========================================================================*/

#define MAIN_SLEEP     100 // in milliseconds

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

//void SendUint8ToComputer(uint8_t* data, uint16_t size) 
//{
//    chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
//    chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
//    chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
//}

//static void serial_start(void)
//{
//    static SerialConfig ser_cfg = {
//        115200,
//        0,
//        0,
//        0,
//    };
//
//    sdStart(&SD3, &ser_cfg); // UART3.
//}

/**
 * @brief 
 * 
 */
static void init_all(void){
    halInit();
    chSysInit();
    mpu_init();

    motors_init();
    mic_create_thd();
    //mic_stop_thd();
    //dist_init();
    //player_init();

    
    //starts the serial communication
    //serial_start();
    //start the USB communication
    //usb_start();
    //starts the camera
    //dcmi_start();
    //po8030_start();
    //inits the motors
/*
    //stars the threads for the lfr regulator and the processing of the image
    lfr_regulator_start();
    image_processing_start();
*/
}

int main(void)
{
    init_all();
    //create_mode_thd(); not working for now

    while(true){
        chThdSleepMilliseconds(MAIN_SLEEP);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}