/**
 * @file ir_sensors.c
 * @brief 
 */

// C standard header files
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// ChibiOS headers
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include "msgbus/messagebus.h"
#include "chprintf.h"
#include <usbcfg.h>

// e-puck 2 main processor headers
#include "sensors/VL53L0X/VL53L0X.h"
#include "sensors/proximity.h"

// Module headers 
#include <ir_sensors.h>
#include <communication.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define TOF_PERIOD          100
#define IR_PERIOD           100

#define NB_AVG              3

/*===========================================================================*/
/* Bus related declarations.                                                 */
/*===========================================================================*/

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static uint16_t dist1_mm = 0u;
static uint16_t dist2_mm = 0u;
static uint16_t dist3_mm = 0u;
static proximity_msg_t prox_values = {0u};

/*===========================================================================*/
/* Module thread pointers.                                                   */
/*===========================================================================*/

static thread_t* ptr_tof_thd = NULL;
static thread_t* ptr_ir_thd = NULL;

/*===========================================================================*/
/* Module threads.                                                           */
/*===========================================================================*/

static THD_WORKING_AREA(wa_tof_thd, 128);
static THD_FUNCTION(tof_thd, arg)
{
    chRegSetThreadName(__FUNCTION__);
	(void)arg;

    while (chThdShouldTerminateX() == false) {
        dist3_mm = dist2_mm;
        dist2_mm = dist1_mm;
        dist1_mm = VL53L0X_get_dist_mm();
        chThdSleepMilliseconds(TOF_PERIOD);
    }

    chThdExit(0);
}

static void tof_create_thd(void)
{
    ptr_tof_thd = chThdCreateStatic(wa_tof_thd, sizeof(wa_tof_thd),
	                                NORMALPRIO, tof_thd, NULL);
}

static THD_WORKING_AREA(wa_ir_thd, 128);
static THD_FUNCTION(ir_thd, arg)
{
    chRegSetThreadName(__FUNCTION__);
	(void)arg;

    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");

    while (chThdShouldTerminateX() == false){
        messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
        chThdSleepMilliseconds(IR_PERIOD);
    }

    chThdExit(0);
}

static void ir_create_thd(void)
{
    ptr_ir_thd = chThdCreateStatic(wa_ir_thd, sizeof(wa_ir_thd),
	                                NORMALPRIO, ir_thd, NULL);
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void sensors_init(void)
{
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	proximity_start();
	VL53L0X_start();
	tof_create_thd();
    ir_create_thd();
    calibrate_ir();
}

uint16_t get_ir_delta(ir_id_t ir_number)
{
    return prox_values.delta[ir_number];
}

uint16_t get_tof_dist(void)
{
    return (dist1_mm + dist2_mm + dist3_mm)/NB_AVG;
}
