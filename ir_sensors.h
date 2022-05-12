/**
 * @file ir_sensors.h
 * @brief 
 */

#ifndef _SENSORS_H_
#define _SENSORS_H_

/*===========================================================================*/
/*  Module data structures and types                                         */
/*===========================================================================*/

typedef enum ir_id_t {
    IR1 = 0,
    IR2,
    IR3,
    IR4,
    IR5,
    IR6,
    IR7,
    IR8,
} ir_id_t;

/*========================================================================*/
/*  External declarations                                                 */
/*========================================================================*/

void sensors_init(void);

uint16_t get_tof_dist(void);
uint16_t get_ir_delta(ir_id_t ir_number);

#endif /* _SENSORS_H_ */
