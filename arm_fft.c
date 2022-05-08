/**
 * @file    arm_fft.c
 * @brief   e-puck 2 fft optimized for arm
 */

// C standard header files
#include <math.h>

// ChibiOS headers
#include "hal.h"
#include "ch.h"

// ARM headers
#include <arm_math.h>
#include <arm_const_structs.h>

// Module headers
#include <mod_arm_fft.h>

/*===========================================================================*/
/*  Module exported functions                                                */
/*===========================================================================*/

/**
 * @brief 
 * 
 * @param size 
 * @param complex_buffer_input
 * @param complex_buffer_output
 */
void doFFT_optimized(uint16_t size, float* complex_buffer_input,
									float* complex_buffer_output){
	if(size == 1024)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer_input, 0, 1);
		arm_cmplx_mag_f32(complex_buffer_input, complex_buffer_output, size);
}

//void doFFT_optimized(uint16_t size, float* complex_buffer){
//	if(size == 1024)
//		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);
//}