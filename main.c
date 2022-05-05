#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>

#include <lfr_regulator.h>
#include <image_processing.h>
#include <distance.h>
#include <player.h>

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
    chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
    chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
    chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
    static SerialConfig ser_cfg = {
        115200,
        0,
        0,
        0,
    };

    sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
    po8030_start();
    //inits the motors
    motors_init();
/*
    //stars the threads for the lfr regulator and the processing of the image
    lfr_regulator_start();
    image_processing_start();
*/
    dist_init();
    player_init();

    uint16_t freq = 31;
    while(1) {
        if (dist_obstacle_is_close()) {
            freq = 31;
            //set_led(4, 1);
        } else {
            freq *= 1.1f;
            if (freq > 4978) freq = 31;
            //set_led(4, 0);
        }

        player_set_frequency(freq);
        chThdSleepMilliseconds(100);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
