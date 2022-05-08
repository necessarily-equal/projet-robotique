/**
 * @file state.c
 * @brief 
 */

// C standard headers files
#include <stdio.h>
#include <stdlib.h>

// ChibiOS headers

#include "hal.h"
#include "ch.h"
#include "memory_protection.h"
#include "chprintf.h"
#include <usbcfg.h>

// e-puck 2 main processor headers
#include <selector.h>

// Module headers
#include <operation_mode.h>
#include <move_command.h>

#include <mic_processing.h>
//#include <maze_control.h>
//#include <lfr_regulator.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define IDLE_MODE               0
#define MIC_REMOTE_MODE         1
#define MAZE_SOLVER_MODE        2
#define LFR_MODE                3

#define MODE_PERIOD             100

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief 
 * 
 * @param mode 
 */
void launch_mode(){
    switch (get_selector())
        {
        case IDLE_MODE:
            mic_stop_thd();
            break;
        case MIC_REMOTE_MODE:
            mic_create_thd();
        default:
            mic_stop_thd();
            break;
        }
}

/*===========================================================================*/
/* Module threads                                                            */
/*===========================================================================*/

static THD_WORKING_AREA(wa_mode_thd, 1024);
static THD_FUNCTION(mode_thd, arg)
{
    chRegSetThreadName(__FUNCTION__);
    (void) arg;

    while (true){
        launch_mode();
        chThdSleepMilliseconds(MODE_PERIOD);
    }
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void create_mode_thd(void)
{
    chThdCreateStatic(wa_mode_thd, sizeof(wa_mode_thd), NORMALPRIO+1,
                      mode_thd, NULL);
}
