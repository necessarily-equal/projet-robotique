/**
 * @file    mic_processing.c
 * @brief   
 */

// ChibiOS headers
#include "hal.h"
#include "ch.h"
#include <usbcfg.h>
#include "memory_protection.h"
#include "msgbus/messagebus.h"

// e-puck 2 main processor headers
#include <audio/microphone.h>
#include "leds.h"

// ARM headers
#include <arm_math.h>

// Module headers
#include <mic_processing.h>
#include <arm_fft.h>
#include <maze_control.h>
#include <move_command.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define MIC_THD_PERIOD      	100     // Refresh @ 10 Hz.

//FFT constants
#define MIN_VALUE_THRESHOLD		35000
#define FFT_SIZE 				1024

//Reduce the frequency range for efficency
#define MIN_FREQ        		10
#define MAX_FREQ        		30

//Frequencies attributed to command
#define FREQ_WAIT       		12  //188Hz
#define FREQ_MOVE       		14  //219Hz
#define FREQ_RST_MAZE			16  //250Hz
#define FREQ_START_MAZE 		18  //281Hz
#define FREQ_U_TURN     		20  //312Hz
#define FREQ_TURN_LEFT  		22  //344Hz
#define FREQ_TURN_RIGHT 		24  //375Hz
#define FREQ_SLOW_DOWN  		26  //406Hz
#define FREQ_SPEED_UP   		28  //437Hz

//Frequencies ranges (attributed to center frequency plus minus one)
#define FREQ_WAIT_LOW           (FREQ_WAIT-1)
#define FREQ_WAIT_HIGH          (FREQ_WAIT+1)
#define FREQ_MOVE_LOW           (FREQ_MOVE-1)
#define FREQ_MOVE_HIGH          (FREQ_MOVE+1)
#define FREQ_START_MAZE_LOW     (FREQ_START_MAZE-1)
#define FREQ_START_MAZE_HIGH    (FREQ_START_MAZE+1)
#define FREQ_RST_MAZE_LOW      	(FREQ_RST_MAZE-1)
#define FREQ_RST_MAZE_HIGH     	(FREQ_RST_MAZE+1)
#define FREQ_U_TURN_LOW         (FREQ_U_TURN-1)
#define FREQ_U_TURN_HIGH        (FREQ_U_TURN+1)
#define FREQ_TURN_LEFT_LOW      (FREQ_TURN_LEFT-1)
#define FREQ_TURN_LEFT_HIGH     (FREQ_TURN_LEFT+1)
#define FREQ_TURN_RIGHT_LOW     (FREQ_TURN_RIGHT-1)
#define FREQ_TURN_RIGHT_HIGH    (FREQ_TURN_RIGHT+1)
#define FREQ_SLOW_DOWN_LOW      (FREQ_SLOW_DOWN-1)
#define FREQ_SLOW_DOWN_HIGH     (FREQ_SLOW_DOWN+1)
#define FREQ_SPEED_UP_LOW       (FREQ_SLOW_DOWN-1)
#define FREQ_SPEED_UP_HIGH      (FREQ_SLOW_DOWN+1)

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

//2 times FFT_SIZE because these arrays contain complex numbers
static float micLeft_cmplx_input[2 * FFT_SIZE];
//static float micRight_cmplx_input[2 * FFT_SIZE];
//static float micFront_cmplx_input[2 * FFT_SIZE];
//static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
//static float micRight_output[FFT_SIZE];
//static float micFront_output[FFT_SIZE];
//static float micBack_output[FFT_SIZE];

//Default command state
static command_t status=WAIT_CMD;

//Thread state
static bool is_recording = false;
static bool is_paused = false;

/*===========================================================================*/
/* Module thread pointers                                                    */
/*===========================================================================*/

static thread_t *ptr_mic_thd = NULL;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

void mic_remote(float* data){
    float max_norm = MIN_VALUE_THRESHOLD;
    int16_t max_norm_index = -1;

    //search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}

    //Simple state machine
    switch(status){
        case IDLE_CMD:
            if(max_norm_index >= FREQ_WAIT_LOW &&
               max_norm_index <= FREQ_WAIT_HIGH){
				status=WAIT_CMD;
               }
			else if(max_norm_index >= FREQ_MOVE_LOW &&
               max_norm_index <= FREQ_MOVE_HIGH){
				status=MOVE_CMD;
               }
            else if(max_norm_index >= FREQ_START_MAZE_LOW &&
               max_norm_index <= FREQ_START_MAZE_HIGH){
				status=START_MAZE_CMD;
               }
            else if(max_norm_index >= FREQ_RST_MAZE_LOW &&
               max_norm_index <= FREQ_RST_MAZE_HIGH){
				status=RST_MAZE_CMD;
               }
            else if(max_norm_index >= FREQ_U_TURN_LOW &&
               max_norm_index <= FREQ_U_TURN_HIGH){
				status=U_TURN_CMD;
               }
            else if(max_norm_index >= FREQ_TURN_LEFT_LOW &&
               max_norm_index <= FREQ_TURN_LEFT_HIGH){
				status=TURN_LEFT_CMD;
               }
            else if(max_norm_index >= FREQ_TURN_RIGHT_LOW &&
               max_norm_index <= FREQ_TURN_RIGHT_HIGH){
				status=TURN_RIGHT_CMD;
               }
            else if(max_norm_index >= FREQ_SLOW_DOWN_LOW &&
               max_norm_index <= FREQ_SLOW_DOWN_HIGH){
				status=SLOW_DOWN_CMD;
               }
            else if(max_norm_index >= FREQ_SPEED_UP_LOW &&
               max_norm_index <= FREQ_SPEED_UP_HIGH){
				status=SPEED_UP_CMD;
               }
            break;

    case WAIT_CMD:
		right_angle_turn(CLOCKWISE);
        status = IDLE_CMD;
        break;
    case MOVE_CMD:
		right_angle_turn(CLOCKWISE);
        status = IDLE_CMD;
        break;
    case START_MAZE_CMD:
		right_angle_turn(CLOCKWISE);
        status = IDLE_CMD;
        break;
    case RST_MAZE_CMD:
		right_angle_turn(CLOCKWISE);
        status = IDLE_CMD;
        break;
    case U_TURN_CMD:
		right_angle_turn(CLOCKWISE);
        status = IDLE_CMD;
        break;
    case TURN_LEFT_CMD:
		right_angle_turn(COUNTERCLOCKWISE);
        status = IDLE_CMD;
        break;
    case TURN_RIGHT_CMD:
		right_angle_turn(COUNTERCLOCKWISE);
        status = IDLE_CMD;
        break;
    case SLOW_DOWN_CMD:
		right_angle_turn(COUNTERCLOCKWISE);
        status = IDLE_CMD;
        break;
    case SPEED_UP_CMD:
		right_angle_turn(COUNTERCLOCKWISE);
        status = IDLE_CMD;
        break;
	default:
		status = IDLE_CMD;
		break;
	}
}

/**
 * @brief               audio processing function taken from TP 5
 * 
 * @param data          Buffer containing 4 times 160 mic samples.
 *                      The samples are directly sorted by the micro.
 * @param num_samples   Tells how many data we get in total (tpy:640 see above)
 */
void process_audio_data(int16_t *data, uint16_t num_samples){
    /*  We get 160 samples per mic every 10ms. So we fill the samples buffers
     *  to reach 1024 samples, then we compute the FFTs.
     *  Sw we fill the samples buffers to reach
     */
	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;
	
	if(is_paused && is_recording){
	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		//micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		//micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		//micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		//micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		//micBack_cmplx_input[nb_samples] = 0;
		//micFront_cmplx_input[nb_samples] = 0;

		//nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		//doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input, micLeft_output);
		//doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		//doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		//arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		//arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		//arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		//arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		nb_samples = 0;
		mustSend++;

		mic_remote(micLeft_output);
	}
	}
}

/*===========================================================================*/
/* Module threads.                                                           */
/*===========================================================================*/

static THD_WORKING_AREA(wa_mic_processing, 1024);
static THD_FUNCTION(thd_mic_processing, arg){
	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	systime_t time;

	while(!chThdShouldTerminateX()){
		//Thread Sleep
		chSysLock();
		if (is_paused){
			chSchGoSleepS(CH_STATE_SUSPENDED);
		}
		chSysUnlock();
		//Thread loop function
		time = chVTGetSystemTime();
		mic_start(&process_audio_data);
		chThdSleepUntilWindowed(time, time + MS2ST(MIC_THD_PERIOD));
	}

	is_recording = false;
	chThdExit(0);
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/




/**
 * @brief 
 * 
 */
void mic_create_thd(void){
	if(!is_recording){
		ptr_mic_thd = chThdCreateStatic(wa_mic_processing, sizeof(wa_mic_processing),
                      NORMALPRIO, thd_mic_processing, NULL);
		is_recording = true;
	}
}

/**
 * @brief 
 * 
 */
void mic_stop_thd(void){
	if (is_recording){
		mic_resume_thd();
		chThdTerminate(ptr_mic_thd);
		chThdWait(ptr_mic_thd);
		is_recording = false;
		is_paused = false;
	}
}

/**
 * @brief 
 * 
 */
void mic_pause_thd(void){
	if(is_recording){
		is_paused = true;
	}
}

/**
 * @brief 
 * 
 */
void mic_resume_thd(void){
	chSysLock();
	if(is_recording && is_paused){
		chSchWakeupS(ptr_mic_thd, CH_STATE_READY);
		is_paused = false;
	}
	chSysUnlock();
}
