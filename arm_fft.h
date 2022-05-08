/**
 * @file    arm_fft.h
 * @brief   FFT optimized for ARM based e-puck 2, taken from TP 5
 */

#ifndef _ARM_FFT_H_
#define _ARM_FFT_H_

/*========================================================================*/
/*  External declarations                                                 */
/*========================================================================*/

/**
 * @brief 
 * 
 * @param size 
 * @param complex_buffer_input 		// modified !
 * @param complex_buffer_output 
 */
void doFFT_optimized(uint16_t size, float* complex_buffer_input,
									float* complex_buffer_output);

#endif /* _ARM_FFT_H_ */