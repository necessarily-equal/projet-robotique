/**
 * @file communication.c
 * @brief
 */

#include <stdint.h>

// ChibiOS headers

#include "hal.h"
#include "ch.h"
#include <usbcfg.h>
#include "chprintf.h"

// Module headers

#include <communication.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define SERIAL_BIT_RATE			115200
#define MAX_BUFFER_SIZE			1024

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void com_serial_start(void)
{
	static SerialConfig ser_cfg = {
		SERIAL_BIT_RATE,
		0,
		0,
		0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

void com_send_data(BaseSequentialStream* out, uint8_t* data, uint16_t size)
{
    uint16_t k = 0;
    uint16_t length = size;
    if (size > MAX_BUFFER_SIZE) {
        while (length > MAX_BUFFER_SIZE) {
            chSequentialStreamWrite(out, data + k*MAX_BUFFER_SIZE,
                                    sizeof(uint8_t) * MAX_BUFFER_SIZE);
            length -= MAX_BUFFER_SIZE;
            ++k;
        }
    }
    chSequentialStreamWrite(out, data + k*MAX_BUFFER_SIZE,
                            sizeof(uint8_t) * length);
}