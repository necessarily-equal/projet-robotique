/**
 * @file    mic_remote_control.c
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
#include <selector.h>

// ARM headers
#include <arm_math.h>

// Module headers
#include "action_queue.h"
#include "mic_remote_control.h"
#include "arm_fft.h"
#include "move_command.h"

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define MIC_SELECTOR_PERIOD		1000	// [ms]

//FFT constants
#define MIN_VALUE_THRESHOLD		35000
#define FFT_SIZE 				1024

//Reduce the frequency range for efficency
#define MIN_FREQ        		10
#define MAX_FREQ        		30

//Frequencies attributed to command
#define FREQ_U_TURN     		20  //312Hz
#define FREQ_TURN_LEFT  		22  //344Hz
#define FREQ_TURN_RIGHT 		24  //375Hz
#define FREQ_STRAIGHT    		26  //406Hz

//Frequencies ranges (attributed to center frequency plus minus one)
#define FREQ_U_TURN_LOW         (FREQ_U_TURN-1)
#define FREQ_U_TURN_HIGH        (FREQ_U_TURN+1)
#define FREQ_TURN_LEFT_LOW      (FREQ_TURN_LEFT-1)
#define FREQ_TURN_LEFT_HIGH     (FREQ_TURN_LEFT+1)
#define FREQ_TURN_RIGHT_LOW     (FREQ_TURN_RIGHT-1)
#define FREQ_TURN_RIGHT_HIGH    (FREQ_TURN_RIGHT+1)
#define FREQ_STRAIGHT_LOW       (FREQ_STRAIGHT-1)
#define FREQ_STRAIGHT_HIGH      (FREQ_STRAIGHT+1)

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

//Thread state
static bool selector_thd_active = false;
static bool selector_thd_paused = false;

//Mic disable flag
static bool disable_mic = true;	// disable callback function

/*===========================================================================*/
/* Module thread pointers                                                    */
/*===========================================================================*/

static thread_t *ptr_mic_selector_thd = NULL;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

static action_t identify_frequency(uint16_t freq) {
	if(freq >= FREQ_U_TURN_LOW &&
	   freq <= FREQ_U_TURN_HIGH){
		return ACTION_BACK;
	}
	else if(freq >= FREQ_TURN_LEFT_LOW &&
	   freq <= FREQ_TURN_LEFT_HIGH){
		return ACTION_LEFT;
	}
	else if(freq >= FREQ_TURN_RIGHT_LOW &&
	   freq <= FREQ_TURN_RIGHT_HIGH){
		return ACTION_RIGHT;
	}
	else if(freq >= FREQ_STRAIGHT_LOW &&
	   freq <= FREQ_STRAIGHT_HIGH){
		return ACTION_STRAIGHT;
	}
	return ACTION_VOID;
}

void mic_remote(float* data){
    float max_norm = MIN_VALUE_THRESHOLD;
    int16_t max_norm_index = 0;

    //search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	static action_t last_identified_frequencies[5] = {};
	static size_t last_identified_frequencies_index = 0;
	static const size_t last_identified_frequencies_len = sizeof(last_identified_frequencies) / sizeof(*last_identified_frequencies);

	last_identified_frequencies[last_identified_frequencies_index++] = identify_frequency(max_norm_index);
	last_identified_frequencies_index %= last_identified_frequencies_len;

	for (size_t i = 1; i < last_identified_frequencies_len; i++) {
			if (last_identified_frequencies[0] != last_identified_frequencies[i])
					return; // nothing to do, not all the previous freq are equal
	}

	// last_identified_frequencies are all equal

	static action_t last_added_action = ACTION_VOID;

	if (last_identified_frequencies[0] != last_added_action) {
		last_added_action = last_identified_frequencies[0];
		action_queue_push(last_added_action);
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

	if(!disable_mic){
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

static THD_WORKING_AREA(wa_mic_selector_thd, 128);
static THD_FUNCTION(thd_mic_selector, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	systime_t time;

	mic_start(&process_audio_data);

	while(!chThdShouldTerminateX()){
		//Thread Sleep
		chSysLock();
		if (selector_thd_paused){
			chSchGoSleepS(CH_STATE_SUSPENDED);
		}
		chSysUnlock();
		//Thread function
		disable_mic = !!(get_selector() & (1<<3));

		//Thread refresh rate
		time = chVTGetSystemTime();
		chThdSleepUntilWindowed(time, time + MS2ST(MIC_SELECTOR_PERIOD));
	}

	selector_thd_active = false;
	chThdExit(0);
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void create_mic_selector_thd(void)
{
	if(!selector_thd_active){
		ptr_mic_selector_thd = chThdCreateStatic(wa_mic_selector_thd,
			sizeof(wa_mic_selector_thd), NORMALPRIO, thd_mic_selector, NULL);
		selector_thd_active = true;
	}
}

void stop_mic_selector_thd(void)
{
	if (selector_thd_active){
		resume_mic_selector_thd();
		chThdTerminate(ptr_mic_selector_thd);
		chThdWait(ptr_mic_selector_thd);
		selector_thd_active = false;
		selector_thd_paused = false;
	}
}

void pause_mic_selector_thd(void)
{
	if(selector_thd_active){
		selector_thd_paused = true;
	}
}

void resume_mic_selector_thd(void)
{
	chSysLock();
	if(selector_thd_active && selector_thd_paused){
		chSchWakeupS(ptr_mic_selector_thd, CH_STATE_READY);
		selector_thd_paused = false;
	}
	chSysUnlock();
}
