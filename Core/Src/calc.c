/*
 * calc.c
 *
 *  Created on: Oct 29, 2023
 *      Author: eb
 */

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "main.h"
#include "global.h"
#include "calc.h"
#include "term.h"

extern uint16_t adc_dma_buf[ADC_NUM][ADC_DMA_BUF_SIZE];
//extern uint16_t adc2_dma_buf[];

extern uint16_t adc_raw_buf[ADC_NUM*ADC_NUM_CHANNELS][ADC_NUM_DATA];	// buffer for 4 channels of raw ADC data
extern uint16_t sample_buf[ADC_NUM_BUFFERS][SAMPLE_BUF_SIZE];			// buffer for 4 channels of down-sampled data
struct sampleBufMeta sample_buf_meta[ADC_NUM_BUFFERS];					// store meta data for associated buffer

//inline int16_t MAX(int16_t a, int16_t b) { return((a) > (b) ? a : b); }
//inline int16_t MIN(int16_t a, int16_t b) { return((a) < (b) ? a : b); }

/*
 * Process the DMA buffer
 *
 * parameter second_half: > 0 to process 2nd half of buffer, 0 = 1st half of buffer
 * parameter adc_num: 0 = ADC1, 1 = ADC2 (use ADC1_IDX or ADC2_IDX)
 * returns: -1 on failure, 0 if OK
 * Split the multi channel readings from the DMA buffer into adc_raw_buf which is
 * structured to hold the readings for one channel per array element.
 * This function also establishes the min/max values for each channel
 * Note: Each ADC is assigned one DMA buffer.
 * One DMA buffer contains 2 data sets, one which is "completed" and
 * one which is currently in use by DMA. The parameter "second_half" indicates which
 * of the two halves is ready for processing (not in use by DMA).
 * The DMA buffer is made up of a sequence of alternate readings (CH0,CH1,CH0,CH1, ....)
 */
int calc_process_dma_buffer(int second_half, int adc_num) {
	uint16_t i = 0;
	uint16_t dma_buf_start, dma_buf_end;		// DMA buffer source
	uint16_t raw_buf_idx = 0;
	uint8_t raw_buf_first, raw_buf_second;		// destination index for raw readings
	// adc_num range check (has to be either ADC1 or ADC2
	if ( (adc_num != ADC1_IDX) && (adc_num != ADC2_IDX) ) {
		return -1;
	}
	// channel index to raw buffer array
	raw_buf_first = adc_num *2;			// destination index for first channel reading in DMA buffer
	raw_buf_second = raw_buf_first + 1; // destination index for second channel reading in DMA buffer
	// first or second half of DMA buffer?
	if (second_half) {
		dma_buf_start = ADC_DMA_BUF_SIZE / 2;
	} else {
		dma_buf_start = 0;	// first half
	}
	dma_buf_end = dma_buf_start + (ADC_DMA_BUF_SIZE / 2) -1;

	// clear meta data in sample buffer
	sample_buf_meta[raw_buf_first].min = ADC_FS_RAW;
	sample_buf_meta[raw_buf_first].max = 0;
	sample_buf_meta[raw_buf_second].min = ADC_FS_RAW;
	sample_buf_meta[raw_buf_second].max = 0;
	sample_buf_meta[raw_buf_first].zero_cross_pos = -1;
	sample_buf_meta[raw_buf_first].zero_cross_neg = -1;
	sample_buf_meta[raw_buf_second].zero_cross_pos = -1;
	sample_buf_meta[raw_buf_second].zero_cross_neg = -1;

	// split DMA buffer in to channels and copy readings into raw buffers
	// step of ADC_NUM_CHANNELS = 2
	for (i=dma_buf_start; i<=dma_buf_end; i+=ADC_NUM_CHANNELS) {
		adc_raw_buf[raw_buf_first][raw_buf_idx] = adc_dma_buf[adc_num][i];		// first entry in DMA buffer
		adc_raw_buf[raw_buf_second][raw_buf_idx++] = adc_dma_buf[adc_num][i+1]; // second entry in DMA buffer

	}
	// down-sample both channels
	calc_downsample(raw_buf_first);
	calc_downsample(raw_buf_second);
	return 0;
}

/*
 * Detect zero point crossings in a buffer and store index in buffer meta data
 * parameter bufnum: the sample buffer to examine
 * parameter zeropoint: the reading which represents the zero point
 * parameter window: zero point window height
 * The challenge with zero crossing is noise contained in the recordings.
 * Whilst some of the noise has been reduced by down-sampling, we have to
 * account for residual noise in the data, hence the somewhat extensive
 * detection method.
 * Detection method: iterate over all points and find the point
 * where one value is above zero threshold and a neighboring point is below
 * then check another point either side of those points to see if
 * they conform to the same slope. Out of those 3 detections at least
 * two being true will yield record a crossing.
 */
void calc_zero_detector(uint8_t bufnum, int zeropoint, int window) {
	if (bufnum >= ADC_NUM_BUFFERS) { return; }
	uint8_t detected = 0;
	uint8_t detect_count = 0;
	// set the detection window
	int window_h = zeropoint + (window/2);
	int window_l = zeropoint - (window/2);

	for (int i=1; i < SAMPLE_BUF_SIZE; i++) {
		// start looking for crossing if reading is within the window
		if ( (sample_buf[bufnum][i] >= window_l) && (sample_buf[bufnum][i] <= window_h) ) {
			// positive slope crossing
			if ( (sample_buf[bufnum][i] > zeropoint) && (sample_buf[bufnum][i-1] <= zeropoint) ) {
				detected++;
				if (i-2 >= 0) {	// check point on lower side
					if (sample_buf[bufnum][i-2] < zeropoint) {
						detected++;
					}
				}
				if (i+2 < SAMPLE_BUF_SIZE) {	// check point on higher side
					if (sample_buf[bufnum][i+2] > zeropoint) {
						detected++;
					}
				}
			}
			if (detected > 1) {	// if at least two of the above checks have a positive result
				sample_buf_meta[bufnum].zero_cross_pos = i;	// set positive crossing point
				detected = 0;
				detect_count++;
				continue;		// back to for loop
			}
			detected = 0;
			// negative slope crossing
			if ( (sample_buf[bufnum][i] < zeropoint) && (sample_buf[bufnum][i-1] >= zeropoint) ) {
				detected++;
				if (i-2 >= 0) {	// check point on lower side
					if (sample_buf[bufnum][i-2] >= zeropoint) {
						detected++;
					}
				}
				if (i+2 < SAMPLE_BUF_SIZE) {	// check point on higher side
					if (sample_buf[bufnum][i+2] < zeropoint) {
						detected++;
					}
				}
				if (detected > 1) {	// if at least two of the above checks have a positive result
					sample_buf_meta[bufnum].zero_cross_neg = i;	// set positive crossing point
					detect_count++;
				}
				detected = 0;
			}
		}	// if inside window
	}	// for in buffer

	// check how many crossings were detected
	if (detect_count > 2) {
		// if we have more than 2 crossings the crossing detections are marked invalid
		sample_buf_meta[bufnum].zero_cross_neg = -9;
		sample_buf_meta[bufnum].zero_cross_pos = -9;
	}
}

/*
 * Down-sample ADC raw readings into sample buffer
 * This function provides a filter for the raw ADC readings. It halves
 * the number of samples and averages adjoining samples to smooth out peaks.
 * It also establishes the meta data (min/max and zero crossing) for both channel
 */
void calc_downsample(uint8_t bufnum) {
	uint16_t range;
	uint16_t dest_idx=0;
	if (bufnum >= ADC_NUM_BUFFERS) { return; }
	for (int i=1; i < ADC_NUM_DATA; i+=2) {
		// calculate reading value by averaging 3 readings (the one before and the one after)
		sample_buf[bufnum][dest_idx] = (adc_raw_buf[bufnum][i] + adc_raw_buf[bufnum][i-1] + adc_raw_buf[bufnum][i+1]) / 3;
		// track min/max values
		sample_buf_meta[bufnum].min = MIN(sample_buf_meta[bufnum].min, sample_buf[bufnum][dest_idx]);
		sample_buf_meta[bufnum].max = MAX(sample_buf_meta[bufnum].max, sample_buf[bufnum][dest_idx]);
		dest_idx++;
	}
	// range of readings
	range = sample_buf_meta[bufnum].max - sample_buf_meta[bufnum].min;
	// detect zero crossings
	calc_zero_detector(bufnum, range / 2 + sample_buf_meta[bufnum].min, range/5);
}

/*
 * Show the adc_raw_buf contents in terminal
 */
void calc_show_buffer(uint8_t buf_num) {
	int count = 0;
	uint16_t address = 0;
	uint64_t squared_acc = 0;
	uint16_t rms_value, adc_raw;
	//uint16_t adc_raw_min = adc_raw_buf[buf_num][0];
	//uint16_t adc_raw_max = adc_raw_min;
	if (buf_num >= ADC_NUM_BUFFERS) { return; }
	term_print("Buffer %d\r\n", buf_num);
	term_print("%3d: ", 0);
	for (int i=0; i<ADC_NUM_DATA; i++) {
		if (count >= 20) {
			count =0;
			term_print("\r\n%3d: ", address);
		}
		adc_raw = adc_raw_buf[buf_num][i];
		term_print("%04u ", adc_raw);

		squared_acc += adc_raw_buf[buf_num][i] * adc_raw_buf[buf_num][i];
		count++; address++;
	}
	rms_value = (uint16_t) sqrt((squared_acc / ADC_NUM_DATA));
	term_print("\r\nMin: %dmV Max: %dmV ", calc_adc_raw_to_mv_int(sample_buf_meta[buf_num].min), calc_adc_raw_to_mv_int(sample_buf_meta[buf_num].max) );
	term_print("RMS: %dmV [%u]\r\n", calc_adc_raw_to_mv_int(rms_value), rms_value);
	term_print("Zero crossing: pos=%d neg=%d\r\n", sample_buf_meta[buf_num].zero_cross_pos, sample_buf_meta[buf_num].zero_cross_neg);

}

/*
 * Output adc_raw_buf contents in CSV format to terminal
 */
void calc_csv_buffer(uint8_t buf_num) {
	if (buf_num > 3) { return; }
	term_print("Buffer %d\r\n", buf_num);
	for (int i=0; i<ADC_NUM_DATA; i++) {
		term_print("%d,%u\r\n", i, adc_raw_buf[buf_num][i]);
	}
	term_print("\r\n\r\n");
}

/*
 * Convert ADC raw reading to mv
 * returns mv as int
 */
int calc_adc_raw_to_mv_int(uint16_t adc_raw) {
	return round(calc_adc_raw_to_mv_float(adc_raw));
}

/*
 * Convert raw reading to mV
 */
float calc_adc_raw_to_mv_float(uint16_t adc_raw) {
	return ((float)adc_raw / (float)ADC_FS_RAW) * (float)ADC_FS_MV;
}
