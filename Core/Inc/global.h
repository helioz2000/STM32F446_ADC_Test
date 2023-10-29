/*
 * global.h
 *
 *  Created on: Oct 26, 2023
 *      Author: eb
 */

#ifndef INC_GLOBAL_H_
#define INC_GLOBAL_H_

// debug pin - to monitor ADC conversion trigger frequency via oscilloscope
#define ADC_DEBUG_PORT GPIOA
#define ADC_DEBUG_PIN GPIO_PIN_9

#define LED2_PORT GPIOA
#define LED2_PIN GPIO_PIN_5

/*
 * DMA buffer definition
 * The DMA buffer contains one int16 entry per ADC reading
 * The sequence order is: CH1, CH2, CH1, CH2, CH1, CH2
 * 800 recording for 2 channel require 1600 entries
 * The complete DMA buffer stores 2 sets of recordings
 * Our code received a callback after the first half of the DMA
 * buffer is filled so it can process that data while the DMA fills the
 * second half.
 * Sampling at 40kHzA a full sine wave (20ms) contains 800 samples
 * one sample is taken every 25us or every 0.45 Degrees
 */
#define ADC_NUM_CHANNELS 2U	// number of channels to convert
#define ADC_NUM_DATA 800U		// number of data points to record
#define ADC_BUF_SIZE ADC_NUM_DATA * ADC_NUM_CHANNELS // buffer size
#define ADC_DMA_BUF_SIZE ADC_BUF_SIZE * 2	// DMA buffer contains 2 sets of data

#endif /* INC_GLOBAL_H_ */
