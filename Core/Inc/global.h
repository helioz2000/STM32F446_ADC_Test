/*
 * global.h
 *
 *  Created on: Oct 26, 2023
 *      Author: eb
 */

#ifndef INC_GLOBAL_H_
#define INC_GLOBAL_H_

struct rawBufMeta {
	uint16_t min;
	uint16_t max;
	int zero_cross1;
	int zero_cross2;
};

#define DISPLAY_X DISPL_HEIGHT	// 480
#define DISPLAY_Y DISPL_WIDTH	// 320

#define ADC_FS_RAW 4095		// full scale 12 bit ADC raw reading
#define ADC_FS_MV 3300		// millivolt at full scale
#define ADC_CENTER_RAW 1861	// centre of signal (1500mV = 1861, 1650mV = 2047)

#define ADC_NUM 2		// number of ADCs in use
#define ADC1_IDX 0			// ADC1 index for arrays
#define ADC2_IDX 1			// ADC2 index for arrays

#define ADC_CH_V 0
#define ADC_CH_I1 1
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
#define ADC_NUM_CHANNELS 2	// number of channels to convert (on one ADC)
#define ADC_NUM_DATA 840U		// number of data points to record
#define ADC_BUF_SIZE ADC_NUM_DATA * ADC_NUM_CHANNELS // buffer size
#define ADC_DMA_BUF_SIZE ADC_BUF_SIZE * 2	// DMA buffer contains 2 sets of data

#define MAX(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#define MIN(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })

#endif /* INC_GLOBAL_H_ */
