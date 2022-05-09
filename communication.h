/**
 * @file    mod_communication.h
 * @brief   E-puck 2 communication to PC header.
 */

#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/**
 * @brief                Starts serial communication.
 * @note                 UART3 is connected to Bluetooth.
 */
void com_serial_start(void);

/**
 * @brief 
 * 
 * @param out 
 * @param data 
 * @param size 
 */
void com_send_data(BaseSequentialStream* out, uint8_t* data, uint16_t size);

#endif /* _COMMUNICATION_H_ */
